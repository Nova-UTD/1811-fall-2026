#!/usr/bin/env bash
# Check the Karbon <-> Jetson link after setup. READ-ONLY; no sudo needed.
# Run on either machine's host (not inside Docker):   bash scripts/check_link.sh
# Works out which machine it's on from the cable address. Prints PASS/FAIL per
# check and what to do about each FAIL. Guide: docs/camera_link_setup.md.

KARBON_IP=192.168.100.1
JETSON_IP=192.168.100.2
fails=0
pass() { printf '  PASS  %s\n' "$1"; }
fail() { printf '  FAIL  %s\n        -> %s\n' "$1" "$2"; fails=$((fails + 1)); }
info() { printf '  ....  %s\n' "$1"; }

if ip -4 addr | grep -q "$KARBON_IP/"; then
    role=karbon; peer=$JETSON_IP
elif ip -4 addr | grep -q "$JETSON_IP/"; then
    role=jetson; peer=$KARBON_IP
else
    echo "Neither $KARBON_IP nor $JETSON_IP is on this machine -- is the cable connection up?"
    exit 1
fi
echo "== $(hostname): $role"

echo "-- cable"
if ping -c 2 -W 2 "$peer" >/dev/null 2>&1; then
    pass "reach the other machine ($peer)"
else
    fail "reach the other machine ($peer)" "check the cable and that the other machine is on"
fi

echo "-- network buffers"
check_min() {
    local key=$1 min=$2 val
    val=$(sysctl -n "$key" 2>/dev/null || echo 0)
    if [ "$val" -ge "$min" ]; then pass "$key = $val"
    else fail "$key = $val (want >= $min)" "run scripts/setup_link_$role.sh"; fi
}
check_min net.core.rmem_max 8388608
check_min net.core.wmem_max 8388608
check_min net.ipv4.ipfrag_high_thresh 134217728

echo "-- time sync"
if ! command -v chronyc >/dev/null; then
    fail "chrony installed" "run scripts/setup_link_$role.sh"
elif [ "$role" = jetson ]; then
    if chronyc -n sources 2>/dev/null | grep -q "^\^\* $KARBON_IP"; then
        pass "following the Karbon's clock"
        info "$(chronyc -n tracking | grep 'System time' | sed 's/  */ /g')"
    else
        fail "following the Karbon's clock" "chrony hasn't selected $KARBON_IP -- wait a minute, then 'chronyc sources'; on the Karbon check the firewall rule"
    fi
else
    if chronyc -n tracking 2>/dev/null | grep -q 'Leap status *: Normal'; then
        pass "Karbon clock synced ($(chronyc -n tracking | grep -oP 'Reference ID *: \K.*'))"
    else
        fail "Karbon clock synced" "no time source yet -- check internet, or wait; it still serves the Jetson (local stratum 10)"
    fi
    if [ -f /etc/chrony/conf.d/1811-serve-jetson.conf ]; then pass "serving time to the cable"
    else fail "serving time to the cable" "run scripts/setup_link_karbon.sh"; fi
fi

echo "-- ROS settings"
if [ "$role" = karbon ]; then
    compose="$(cd "$(dirname "$0")/.." && pwd)/docker-compose.yml"
    if grep -qE '^\s*- FASTRTPS_DEFAULT_PROFILES_FILE=/vehicle_1811/config/fastdds_link.xml' "$compose"; then
        pass "docker-compose.yml uses config/fastdds_link.xml"
    else
        fail "docker-compose.yml uses config/fastdds_link.xml" "git pull the branch with the camera link changes"
    fi
    container=$(docker ps --format '{{.Names}}' 2>/dev/null | grep -m1 vehicle_1811)
    if [ -n "$container" ]; then
        prof=$(docker exec "$container" printenv FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null)
        if [ -n "$prof" ]; then pass "running container $container has the profile"
        else fail "running container $container has the profile" "recreate it: docker compose up -d --force-recreate dev"; fi
    else
        info "no vehicle_1811 container running (start one with ./scripts/dev.sh)"
    fi
    if systemctl is-active --quiet ufw; then
        info "firewall is on -- the rule for $JETSON_IP needs sudo to list: sudo ufw status | grep $JETSON_IP"
    fi
else
    for pair in "ROS_DOMAIN_ID=0" "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"; do
        if grep -qE "^\s*export\s+$pair(\s|$)" "$HOME/.bashrc"; then pass "~/.bashrc: $pair"
        else fail "~/.bashrc: $pair" "run scripts/setup_link_jetson.sh"; fi
    done
    prof=$(grep -oP '^\s*export\s+FASTRTPS_DEFAULT_PROFILES_FILE=\K\S+' "$HOME/.bashrc")
    if [ -n "$prof" ] && [ -f "$prof" ]; then pass "~/.bashrc: profile $prof"
    else fail "~/.bashrc: FASTRTPS_DEFAULT_PROFILES_FILE points at a real file" "run scripts/setup_link_jetson.sh"; fi
    if [ -f "$(cd "$(dirname "$0")/.." && pwd)/ros2_ws/install/jetson_bringup/share/jetson_bringup/launch/zed_cameras.launch.py" ]; then
        pass "jetson_bringup built"
    else
        fail "jetson_bringup built" "run scripts/setup_link_jetson.sh (step 4 builds it)"
    fi
fi

echo
if [ "$fails" -eq 0 ]; then
    echo "All checks passed. ROS check: on the other machine, 'ros2 topic list' should show this one's topics."
else
    echo "$fails check(s) failed -- see the -> lines above."
fi
exit "$fails"
