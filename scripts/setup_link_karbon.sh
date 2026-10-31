#!/usr/bin/env bash
# One-time Karbon setup for the Karbon <-> Jetson cable (192.168.100.1 <-> .2).
# Run on the Karbon host, NOT inside Docker:   bash scripts/setup_link_karbon.sh
# Safe to re-run. Lists every change and asks before making any. Needs sudo.
#
# Pair with scripts/setup_link_jetson.sh on the Jetson, then verify both with
# scripts/check_link.sh. Full guide: docs/camera_link_setup.md.
set -euo pipefail

CABLE_IF=enp4s0          # Karbon port the Jetson cable is on ("Orin Connection")
JETSON_IP=192.168.100.2
CABLE_NET=192.168.100.0/24
SYSCTL_FILE=/etc/sysctl.d/99-zz-1811-dds.conf
CHRONY_FILE=/etc/chrony/conf.d/1811-serve-jetson.conf

cat <<EOF
This will set up the Karbon side of the Jetson link:

  1. Firewall (ufw): allow everything arriving on $CABLE_IF from the Jetson
     ($JETSON_IP) -- ROS 2 (DDS) traffic and time sync. Nothing else changes.
  2. Network buffers: $SYSCTL_FILE
     (bigger UDP buffers so camera images aren't dropped).
  3. Time sync: install chrony (replaces systemd-timesyncd) and let it serve
     time to $CABLE_NET: $CHRONY_FILE
     The Karbon keeps syncing to the internet; the Jetson follows the Karbon.

EOF
read -r -p "Continue? [y/N] " answer
[[ "$answer" =~ ^[Yy]$ ]] || { echo "Nothing changed."; exit 0; }

ip -4 addr show "$CABLE_IF" 2>/dev/null | grep -q '192.168.100.1/' || {
    echo "!! $CABLE_IF doesn't have 192.168.100.1 -- is this the Karbon, and is the cable connection up?"
    exit 1
}

echo
echo "== 1/3 Firewall"
if sudo ufw status | grep -q '^Status: active'; then
    sudo ufw allow in on "$CABLE_IF" from "$JETSON_IP" comment '1811 Jetson link'
else
    echo "ufw is not active -- no rule needed."
fi

echo
echo "== 2/3 Network buffers"
# Values from the ROS 2 DDS tuning guide for large messages over a network.
# Named to sort after /etc/sysctl.d/99-ouster.conf, so these larger limits win.
sudo tee "$SYSCTL_FILE" >/dev/null <<'EOF'
# 1811: UDP buffers and IP fragment handling for ROS 2 camera images over the
# Karbon <-> Jetson cable. Written by scripts/setup_link_karbon.sh.
net.core.rmem_max=8388608
net.core.wmem_max=8388608
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sudo sysctl --quiet --system
sysctl net.core.rmem_max net.core.wmem_max net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh

echo
echo "== 3/3 Time sync (chrony)"
if ! command -v chronyd >/dev/null; then
    sudo apt-get update -qq
    sudo apt-get install -y chrony
fi
grep -q '^confdir /etc/chrony/conf.d' /etc/chrony/chrony.conf ||
    echo 'confdir /etc/chrony/conf.d' | sudo tee -a /etc/chrony/chrony.conf >/dev/null
sudo tee "$CHRONY_FILE" >/dev/null <<EOF
# 1811: serve time to the Jetson over the cable. Written by scripts/setup_link_karbon.sh.
allow $CABLE_NET
# Keep serving even when the internet is unreachable, so the two clocks stay
# together on the car (stratum 10 = only used when nothing better exists).
local stratum 10
EOF
sudo systemctl enable --now chrony
sudo systemctl restart chrony
sleep 2
chronyc tracking | grep -E 'Reference ID|System time|Leap status' || true

cat <<EOF

Done. Next:
  - Run scripts/setup_link_jetson.sh on the Jetson.
  - Recreate the Docker container so it picks up config/fastdds_link.xml:
      docker compose up -d --force-recreate dev
  - Check both machines: bash scripts/check_link.sh
EOF
