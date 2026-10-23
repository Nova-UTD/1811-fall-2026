#!/usr/bin/env bash
# One-time: let the Karbon start the Jetson's cameras without a password.
# Run on the Karbon host (not in Docker):   bash scripts/setup_jetson_ssh.sh
#
# Creates a dedicated key (~/.ssh/id_ed25519_jetson -- your other keys are not
# touched), installs it on the Jetson (asks for the Jetson's password once),
# then tests the connection the same way bringup.launch.py uses it.
# Safe to re-run.
set -euo pipefail

JETSON=${1:-nova@192.168.100.2}
KEY="$HOME/.ssh/id_ed25519_jetson"
SSH=(ssh -F /dev/null -i "$KEY" -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=5)

mkdir -p "$HOME/.ssh"
chmod 700 "$HOME/.ssh"

if [ -f "$KEY" ]; then
    echo "== Using the existing key $KEY"
else
    echo "== Creating $KEY (no passphrase, so bringup can use it unattended)"
    ssh-keygen -q -t ed25519 -N '' -C "1811 karbon -> jetson" -f "$KEY"
fi

if "${SSH[@]}" "$JETSON" true 2>/dev/null; then
    echo "== The Jetson already accepts this key."
else
    echo "== Installing the key on $JETSON -- enter the Jetson's password once:"
    ssh-copy-id -i "$KEY.pub" "$JETSON"
fi

echo "== Testing the connection bringup will use..."
if ! "${SSH[@]}" "$JETSON" true; then
    echo "!! Still can't log in without a password. Is the Jetson on, and is 192.168.100.2 reachable (ping -c 2 192.168.100.2)?"
    exit 1
fi
echo "   OK: passwordless login to $JETSON works."

if "${SSH[@]}" "$JETSON" "test -f ~/1811-fall-2026/scripts/jetson_cameras.sh"; then
    echo "   OK: the Jetson has scripts/jetson_cameras.sh."
else
    echo "!! The Jetson's repo doesn't have scripts/jetson_cameras.sh yet -- on the Jetson run:"
    echo "     cd ~/1811-fall-2026 && git pull"
fi

cat <<EOF

Done. Next, in the container (./scripts/dev.sh recreates it so it can see the
key -- close other container terminals first), rebuild once:
  colcon build --symlink-install --packages-ignore umrr_ros2_driver umrr_ros2_msgs smart_rviz_plugin
From then on bringup starts the cameras too:
  ros2 launch obc_bringup bringup.launch.py lidar_ip:=... port:=...                 # cameras
  ros2 launch obc_bringup bringup.launch.py lidar_ip:=... port:=... panorama:=true  # + panorama
EOF
