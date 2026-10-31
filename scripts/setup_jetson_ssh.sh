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
elif "${SSH[@]}" "$JETSON" "git -C ~/1811-fall-2026 rev-parse --is-inside-work-tree >/dev/null 2>&1"; then
    echo "   The Jetson checkout is older; ./g can update it to the Karbon commit."
else
    echo "!! No Jetson checkout at ~/1811-fall-2026; follow docs/camera_link_setup.md first."
fi

cat <<EOF

Done. Close other container terminals, then start bringup from the repo root on
the Karbon host. The shortcut updates and builds the Jetson checkout, rebuilds
the Karbon workspace as needed, and opens RViz:
  ./g                  # cameras + panorama + edge detection + RViz
  ./g panorama:=false  # skip the panorama
EOF
