#!/usr/bin/env bash
# Karbon side of scripts/jetson_cameras.sh: bringup.launch.py runs this to start
# the Jetson's cameras over SSH, and to stop them cleanly when bringup stops.
#
#   bash jetson_cameras_ssh.sh <user@host> <key file> <command> [repo branch commit]
#
# Why not run ssh directly: on Ctrl-C ssh dies at once (exit 255, an [ERROR]
# line in bringup) and the Jetson only finds out from the dropped connection.
# Instead ssh runs in its own session, so the terminal's Ctrl-C doesn't reach
# it, and this script passes the Ctrl-C on by typing it into the Jetson's
# terminal. The Jetson stops its cameras, says so, and ssh exits cleanly.
set -u
host=$1 key=$2 remote=$3

ssh_options=(-F /dev/null -i "$key"
    -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=5
    -o ServerAliveInterval=5 -o ServerAliveCountMax=3
    -o StrictHostKeyChecking=accept-new -o UserKnownHostsFile=/root/.ssh/known_hosts
    -o LogLevel=ERROR)

if (( $# == 6 )); then
    # Send the updater from the Karbon: the Jetson's checkout may be older.
    printf -v update_command 'bash -s -- %q %q %q' "$4" "$5" "$6"
    updater="$(dirname "$0")/sync_jetson_repo.sh"
    if ! timeout 45s ssh -T "${ssh_options[@]}" "$host" "$update_command" <"$updater"; then
        echo '[jetson] [ERROR] Update failed; Jetson cameras and detectors were not started.' >&2
        exit 1
    fi
elif (( $# != 3 )); then
    echo 'usage: jetson_cameras_ssh.sh host key command [repo branch commit]' >&2
    exit 2
fi

fifo=$(mktemp -u) && mkfifo -m 600 "$fifo" || exit 1
# -tt: a terminal on the Jetson, so a typed Ctrl-C (or a dropped connection)
# stops jetson_cameras.sh. -F /dev/null: ~/.ssh is mounted from the host, and
# ssh refuses a config file owned by another user.
setsid ssh -tt "${ssh_options[@]}" "$host" "$remote" <"$fifo" &
ssh_pid=$!
exec 3>"$fifo"     # ssh's keyboard: stays open, and Ctrl-C is written here
rm -f "$fifo"

# Once is enough; bringup sends SIGINT, then SIGTERM if the Jetson is slow.
trap 'trap "" INT TERM HUP; printf "\003" >&3' INT TERM HUP
# wait returns early each time a signal arrives; keep waiting until ssh is done.
while :; do
    wait "$ssh_pid"
    rc=$?
    kill -0 "$ssh_pid" 2>/dev/null || exit "$rc"
done
