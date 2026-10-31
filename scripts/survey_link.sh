#!/usr/bin/env bash
# Collect the facts needed to set up the Karbon <-> Jetson link (static IPs,
# DDS discovery, time sync, ZED cameras). READ-ONLY: changes nothing, needs no
# sudo, and prints no passwords or keys.
#
# Run on BOTH computers, on the host (not inside the Docker container):
#   bash scripts/survey_link.sh
# It prints a report and saves it to ~/link_survey_<hostname>.txt.

out="$HOME/link_survey_$(hostname).txt"

section() { printf '\n===== %s =====\n' "$1"; }
run() { printf '$ %s\n' "$*"; "$@" 2>&1 || true; }

{
section "machine"
run hostname
run uname -m
grep -E '^(PRETTY_NAME|VERSION_ID)=' /etc/os-release
[ -f /etc/nv_tegra_release ] && run head -1 /etc/nv_tegra_release

section "network interfaces (state, addresses)"
run ip -br link
run ip -br addr
section "cable plugged in? (1 = link up) and speed (Mb/s)"
for dev in /sys/class/net/*; do
    name=$(basename "$dev")
    [ "$name" = lo ] && continue
    case "$name" in docker*|br-*|veth*) continue ;; esac
    carrier=$(cat "$dev/carrier" 2>/dev/null || echo "?")
    speed=$(cat "$dev/speed" 2>/dev/null || echo "?")
    echo "$name carrier=$carrier speed=$speed"
done
run ip route
run ip neigh

section "network config tools"
for svc in NetworkManager systemd-networkd; do
    echo "$svc: $(systemctl is-active "$svc" 2>/dev/null)"
done
command -v nmcli >/dev/null && run nmcli -t -f NAME,TYPE,DEVICE,AUTOCONNECT connection show
run ls /etc/netplan
echo "ufw (firewall): $(grep -s '^ENABLED=' /etc/ufw/ufw.conf || echo 'not installed') (service: $(systemctl is-active ufw 2>/dev/null))"

section "time sync"
run timedatectl
for svc in chrony chronyd systemd-timesyncd ptp4l phc2sys; do
    echo "$svc: $(systemctl is-active "$svc" 2>/dev/null)"
done
command -v chronyc >/dev/null && run chronyc tracking
echo "internet (for an upstream time source): $(ping -c 1 -W 2 8.8.8.8 >/dev/null 2>&1 && echo yes || echo no)"

section "ROS on the host"
run ls /opt/ros
env | grep -E '^(ROS_|RMW_|FASTRTPS|FASTDDS|CYCLONEDDS)' || echo "(no ROS variables set in this shell)"
[ -f "$HOME/.bashrc" ] && grep -nE 'ros|ROS_|RMW|FASTRTPS|FASTDDS|zed_ws' "$HOME/.bashrc"

section "docker"
command -v docker >/dev/null && run docker --version
command -v docker >/dev/null && run docker ps --format '{{.Names}}  {{.Image}}  {{.Status}}'

section "ZED SDK and ROS wrapper"
if [ -d /usr/local/zed ]; then
    grep -hE 'define ZED_SDK_(MAJOR|MINOR|PATCH)_VERSION' /usr/local/zed/include/sl/Camera.hpp 2>/dev/null
    run ls /usr/local/zed/tools
else
    echo "(no ZED SDK at /usr/local/zed)"
fi
if [ -d "$HOME/zed_ws/src" ]; then
    run ls "$HOME/zed_ws/src"
    for repo in "$HOME"/zed_ws/src/*/; do
        [ -d "$repo/.git" ] && echo "$(basename "$repo"): $(git -C "$repo" describe --tags --always 2>/dev/null)"
    done
    find "$HOME/zed_ws/src" -path '*launch*' \( -name '*.launch.py' -o -name '*.launch.xml' \) 2>/dev/null | sed "s|$HOME/||"
    find "$HOME/zed_ws/src" -path '*zed_wrapper/config*' -name '*.yaml' 2>/dev/null | sed "s|$HOME/||"
else
    echo "(no ~/zed_ws/src)"
fi
} 2>&1 | tee "$out"

echo
echo "Saved to $out -- paste the whole report back."
