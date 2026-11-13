#!/usr/bin/env bash
# Read-only health report for the 1811 Karbon, and the Jetson over the cable,
# to find out why the car's software crashes. It changes nothing.
#
#   bash scripts/crash_check.sh        # on the Karbon host, not inside Docker
#
# Run it after a crash (before ./reload if you can), then paste the whole
# output to whoever is debugging. It looks for the usual causes:
#   - power cut without a shutdown (files written just before can be lost or
#     left empty, which breaks the next start until a fresh build),
#   - a full disk (builds then write broken files),
#   - running out of memory (the kernel kills the biggest process),
#   - segfaults, GPU hangs, overheating.
set -u
REPO="$(cd "$(dirname "$0")/.." && pwd)"
JETSON=${JETSON:-nova@192.168.100.2}
KEY=${KEY:-$HOME/.ssh/id_ed25519_jetson}

# Everything that runs on both machines. Sent to the Jetson with `declare -f`.
host_report() {
    local name=$1
    j() { journalctl --no-pager -q "$@" 2>/dev/null || sudo -n journalctl --no-pager -q "$@" 2>/dev/null; }
    kgrep() {  # kgrep <boot offset> <pattern> <max lines>: kernel log lines matching
        j -k -b "$1" -o short-iso | grep -iE "$2" | tail -n "$3" | cut -c1-200 | sed 's/^/   /'
    }
    echo "#### $name ($(hostname)): up $(uptime -p 2>/dev/null | sed 's/^up //'), $(date '+%F %T')"

    echo "-- previous boots (\"NO SHUTDOWN\" = the power was cut or it froze; files written just before can be damaged)"
    local boots
    boots=$(j --list-boots | tail -n 8)
    if [ -z "$boots" ]; then
        echo "   (can't read the system journal; run with sudo for this part)"
    elif [ "$(printf '%s\n' "$boots" | wc -l)" -le 1 ]; then
        echo "   (journal only keeps the current boot)"
    else
        printf '%s\n' "$boots" | while read -r off _id rest; do
            [ "$off" = 0 ] && continue
            if j -b "$off" -n 400 -o cat | grep -qiE \
                'Reached target .*(Power-?Off|Reboot|Shutdown|Halt)|systemd-shutdown|Journal stopped|Stopped target'; then
                verdict="shut down cleanly"
            else
                verdict="NO SHUTDOWN"
            fi
            printf '   boot %3s: %-18s %s\n' "$off" "$verdict" "$rest"
        done
    fi
    echo "   last -x (\"crash\" = went down without a shutdown):"
    last -x -F 2>/dev/null | grep -E '^(reboot|shutdown)|crash' | head -n 8 | sed 's/^/     /'

    echo "-- filesystem: repairs at boot and errors (this boot, previous boot)"
    for b in 0 -1; do kgrep "$b" 'EXT4-fs error|EXT4-fs.*(recovery complete|orphan)|I/O error|nvme.*(timeout|reset)' 6; done

    echo "-- disk space"
    df -h / /home 2>/dev/null | awk 'NR == 1 || !seen[$0]++' | sed 's/^/   /'

    echo "-- memory, and processes the kernel killed for memory (this boot, previous boot)"
    free -h | sed 's/^/   /'
    for b in 0 -1; do kgrep "$b" 'out of memory|oom-kill|Killed process' 6; done

    echo "-- crashes: segfaults and similar (this boot, previous boot)"
    for b in 0 -1; do kgrep "$b" 'segfault|general protection|traps:|core dumped' 10; done

    echo "-- GPU hangs/resets and overheating (this boot, previous boot)"
    for b in 0 -1; do kgrep "$b" 'GPU HANG|i915.*(reset|hang|error)|amdgpu.*(timeout|reset)|NVRM|Xid|nvgpu.*(fault|error)|thermal.*(critical|throttl)|temperature above' 6; done
    for z in /sys/class/thermal/thermal_zone*; do
        [ -r "$z/temp" ] && printf '   %s %s C\n' "$(cat "$z/type" 2>/dev/null)" "$(( $(cat "$z/temp") / 1000 ))"
    done | sort -u | head -n 8

    echo "-- shared memory (ROS uses it between programs on one machine)"
    df -h /dev/shm | tail -n 1 | sed 's/^/   /'
    echo "   Fast DDS files: $(ls /dev/shm 2>/dev/null | grep -c fastrtps)"

    echo "-- busiest processes right now (CPU, then memory)"
    ps -eo pid,pcpu,pmem,rss,etime,comm --sort=-pcpu 2>/dev/null | head -n 7 | sed 's/^/   /'
    ps -eo pid,pcpu,pmem,rss,etime,comm --sort=-rss 2>/dev/null | sed -n '2,5p' | sed 's/^/   /'
}

karbon_extra() {
    echo "-- this repo's disk use (backups from ./reload are never deleted automatically)"
    (cd "$REPO" && du -sh ros2_ws/build ros2_ws/install ros2_ws/log ros2_ws/.reload-backups recordings 2>/dev/null) | sed 's/^/   /'
    command -v docker >/dev/null && docker system df 2>/dev/null | sed 's/^/   /'

    echo "-- empty program files in the ROS install (a sign of files lost in a power cut)"
    for d in "$REPO/ros2_ws/install" "$REPO/ros2_ws/.reload-backups"; do
        [ -d "$d" ] || continue
        found=$(find "$d" -type f -size 0 \( -name '*.so*' -o -path '*/lib/*/*' \) ! -name '*.py' ! -name '__init__*' 2>/dev/null)
        printf '   %s: %s empty\n' "${d#"$REPO"/}" "$(printf '%s' "$found" | grep -c .)"
        printf '%s\n' "$found" | grep . | head -n 5 | sed "s|$REPO/|     |"
    done

    echo "-- vehicle containers"
    for c in $(docker ps -a --format '{{.Names}}' 2>/dev/null | grep vehicle_1811); do
        docker inspect -f '   {{.Name}}: {{.State.Status}}, exit {{.State.ExitCode}}, killed for memory: {{.State.OOMKilled}}, restarts {{.RestartCount}}, started {{.State.StartedAt}}' "$c"
    done
    local c="vehicle_1811_dev_${USER}"
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$c"; then
        echo "-- ROS programs that died, from this container's launch logs (newest last)"
        docker exec "$c" bash -c 'grep -h -E "process has died|\[FATAL\]|Segmentation fault|std::bad_alloc|terminate called" /root/.ros/log/*/launch.log 2>/dev/null | tail -n 20' \
            | sed -E 's/, cmd .*/]/' | cut -c1-200 | sed 's/^/   /'
    else
        echo "-- ($c isn't running, so its ROS launch logs can't be read)"
    fi
}

jetson_extra() {
    echo "-- Jetson camera/panorama/calibration programs that died or errored (latest run)"
    grep -h -E "process has died|Traceback|\[FATAL\]|\[ERROR\]|Segmentation fault|CUDA error|out of memory" \
        ~/.ros/jetson_cameras/*_latest.log 2>/dev/null | sed -E 's/, cmd .*/]/' | tail -n 15 | cut -c1-200 | sed 's/^/   /'
    command -v tegrastats >/dev/null && { echo "-- tegrastats (one sample)"; timeout 3 tegrastats --interval 1000 2>/dev/null | head -n 1 | cut -c1-300 | sed 's/^/   /'; }
}

host_report Karbon
karbon_extra
echo
if ssh -F /dev/null -i "$KEY" -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=5 \
        -o LogLevel=ERROR "$JETSON" true 2>/dev/null; then
    { declare -f host_report jetson_extra; echo 'host_report Jetson; jetson_extra'; } |
        ssh -F /dev/null -i "$KEY" -o IdentitiesOnly=yes -o BatchMode=yes -o LogLevel=ERROR "$JETSON" bash -s
else
    echo "#### Jetson: can't log in to $JETSON with $KEY (is it on?). Skipped."
fi
