#!/usr/bin/env bash
# One-time Jetson setup for the Karbon <-> Jetson cable (192.168.100.1 <-> .2).
# Run on the Jetson host (ssh nova@192.168.100.2 from the Karbon), from a clone
# of this repo:   bash scripts/setup_link_jetson.sh
# Safe to re-run. Lists every change and asks before making any. Needs sudo.
#
# Pair with scripts/setup_link_karbon.sh on the Karbon, then verify both with
# scripts/check_link.sh. Full guide: docs/camera_link_setup.md.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
KARBON_IP=192.168.100.1
PROFILE="$REPO/config/fastdds_link.xml"
SYSCTL_FILE=/etc/sysctl.d/99-zz-1811-dds.conf
CHRONY_FILE=/etc/chrony/conf.d/1811-follow-karbon.conf
BASHRC="${BASHRC:-$HOME/.bashrc}"

# Point the ROS environment in a bashrc at the Karbon's settings: domain 0,
# Fast DDS, and the shared cable profile. Edits existing export lines in place
# and appends missing ones. Separate function so it can be tested on its own.
set_ros_env() {
    local file="$1" profile="$2"
    local -A want=(
        [ROS_DOMAIN_ID]=0
        [RMW_IMPLEMENTATION]=rmw_fastrtps_cpp
        [FASTRTPS_DEFAULT_PROFILES_FILE]="$profile"
    )
    local var
    for var in ROS_DOMAIN_ID RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE; do
        if grep -qE "^\s*export\s+$var=" "$file"; then
            sed -i -E "s|^(\s*)export\s+$var=.*|\1export $var=${want[$var]}  # 1811 Karbon link|" "$file"
        else
            echo "export $var=${want[$var]}  # 1811 Karbon link" >>"$file"
        fi
    done
}
[ "${SOURCE_ONLY:-0}" = 1 ] && return 0

cat <<EOF
This will set up the Jetson side of the Karbon link:

  1. ROS settings in $BASHRC (backup: $BASHRC.bak-1811), to match the Karbon:
       ROS_DOMAIN_ID       -> 0                (was: $(grep -oP '^\s*export\s+ROS_DOMAIN_ID=\K\S+' "$BASHRC" || echo unset))
       RMW_IMPLEMENTATION  -> rmw_fastrtps_cpp (was: $(grep -oP '^\s*export\s+RMW_IMPLEMENTATION=\K\S+' "$BASHRC" || echo unset))
       FASTRTPS_DEFAULT_PROFILES_FILE -> $PROFILE
  2. Network buffers: $SYSCTL_FILE (bigger UDP buffers for camera images).
  3. Time sync: install chrony (replaces systemd-timesyncd) and follow the
     Karbon ($KARBON_IP): $CHRONY_FILE
  4. Build jetson_bringup (the camera launch) in $REPO/ros2_ws against ~/zed_ws.

EOF
read -r -p "Continue? [y/N] " answer
[[ "$answer" =~ ^[Yy]$ ]] || { echo "Nothing changed."; exit 0; }

ip -4 addr | grep -q '192.168.100.2/' || {
    echo "!! No interface has 192.168.100.2 -- is this the Jetson, and is the cable connection up?"
    exit 1
}
[ -f "$PROFILE" ] || { echo "!! $PROFILE not found -- run this from the repo clone."; exit 1; }

echo
echo "== 1/4 ROS settings"
[ -d /opt/ros/humble/share/rmw_fastrtps_cpp ] || {
    sudo apt-get update -qq
    sudo apt-get install -y ros-humble-rmw-fastrtps-cpp
}
cp "$BASHRC" "$BASHRC.bak-1811"
set_ros_env "$BASHRC" "$PROFILE"
grep -nE 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|FASTRTPS_DEFAULT_PROFILES_FILE' "$BASHRC"

echo
echo "== 2/4 Network buffers"
sudo tee "$SYSCTL_FILE" >/dev/null <<'EOF'
# 1811: UDP buffers and IP fragment handling for ROS 2 camera images over the
# Karbon <-> Jetson cable. Written by scripts/setup_link_jetson.sh.
net.core.rmem_max=8388608
net.core.wmem_max=8388608
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sudo sysctl --quiet --system
sysctl net.core.rmem_max net.core.wmem_max net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh

echo
echo "== 3/4 Time sync (chrony)"
if ! command -v chronyd >/dev/null; then
    sudo apt-get update -qq
    sudo apt-get install -y chrony
fi
grep -q '^confdir /etc/chrony/conf.d' /etc/chrony/chrony.conf ||
    echo 'confdir /etc/chrony/conf.d' | sudo tee -a /etc/chrony/chrony.conf >/dev/null
sudo tee "$CHRONY_FILE" >/dev/null <<EOF
# 1811: follow the Karbon's clock over the cable. Written by scripts/setup_link_jetson.sh.
# 'prefer' makes chrony pick the Karbon whenever it's reachable; the internet
# pool from chrony.conf stays as a fallback.
server $KARBON_IP iburst prefer
EOF
sudo systemctl enable --now chrony
sudo systemctl restart chrony

echo
echo "== 4/4 Build jetson_bringup"
(
    set +u
    source /opt/ros/humble/setup.bash
    source "$HOME/zed_ws/install/local_setup.bash"
    cd "$REPO/ros2_ws"
    colcon build --symlink-install --packages-select jetson_bringup
)

echo
echo "Waiting for chrony to pick the Karbon (up to 30 s)..."
for _ in $(seq 15); do
    chronyc -n sources | grep -q "^\^\* $KARBON_IP" && break
    sleep 2
done
chronyc -n sources | grep -E "^MS|$KARBON_IP" || true

cat <<EOF

Done. Open a NEW terminal (or run: source ~/.bashrc) so the ROS settings apply,
then stop any old ROS daemon:   ros2 daemon stop
Check both machines:            bash scripts/check_link.sh
Start the cameras:              see docs/camera_link_setup.md
EOF
