#!/usr/bin/env bash
# Sent over SSH by the Karbon, so it also works with an older Jetson checkout.
set -euo pipefail

if (( $# != 3 )); then
    echo '[jetson] [ERROR] Expected the repository, branch, and commit.' >&2
    exit 2
fi
repo=$1 branch=$2 revision=$3
case "$repo" in
    '~') repo=$HOME ;;
    '~/'*) repo="$HOME/${repo#\~/}" ;;
esac

if [[ ! "$revision" =~ ^([[:xdigit:]]{40}|[[:xdigit:]]{64})$ ]] ||
   ! git check-ref-format --branch "$branch" >/dev/null 2>&1; then
    echo '[jetson] [ERROR] Karbon branch or commit is missing; commit its Jetson changes and retry.' >&2
    exit 1
fi
if ! git -C "$repo" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "[jetson] [ERROR] No Git checkout at $repo; complete the one-time Jetson setup." >&2
    exit 1
fi
if ! changes=$(git -C "$repo" status --porcelain --untracked-files=normal); then
    echo '[jetson] [ERROR] Could not check the Jetson working tree.' >&2
    exit 1
fi
if [[ -n "$changes" ]]; then
    echo '[jetson] [ERROR] Jetson checkout has local changes; update skipped to protect them.' >&2
    exit 1
fi

current_branch=$(git -C "$repo" symbolic-ref --quiet --short HEAD || true)
current_revision=$(git -C "$repo" rev-parse --verify HEAD)
if [[ "$current_branch" == "$branch" && "$current_revision" == "$revision" ]]; then
    echo "[jetson] Checkout already matches $branch at ${revision:0:12}."
    exit 0
fi

if ! command -v pgrep >/dev/null 2>&1; then
    echo '[jetson] [ERROR] Cannot check for running camera processes: pgrep is missing.' >&2
    exit 1
fi
if pgrep -f '[r]os2 launch (jetson_bringup zed_cameras.launch.py|camera_panorama panorama.launch.py|lidar_perception corner_detector.launch.py)' >/dev/null; then
    echo '[jetson] [ERROR] Cameras or detectors are already running; stop them before updating the checkout.' >&2
    exit 1
fi

if ! git -C "$repo" cat-file -e "$revision^{commit}" 2>/dev/null; then
    echo "[jetson] Fetching $branch from origin..."
    if ! timeout 30s env GIT_TERMINAL_PROMPT=0 \
        GIT_SSH_COMMAND='ssh -o BatchMode=yes -o ConnectTimeout=5' \
        git -C "$repo" fetch --no-tags --no-recurse-submodules origin "$branch" </dev/null; then
        echo '[jetson] [ERROR] Git fetch failed; check Jetson network access and Git credentials.' >&2
        exit 1
    fi
fi
if ! git -C "$repo" cat-file -e "$revision^{commit}" 2>/dev/null; then
    echo '[jetson] [ERROR] The Karbon commit is not available from the Jetson origin; push it first.' >&2
    exit 1
fi

if git -C "$repo" show-ref --verify --quiet "refs/heads/$branch"; then
    if ! git -C "$repo" merge-base --is-ancestor "refs/heads/$branch" "$revision"; then
        echo '[jetson] [ERROR] Jetson branch has diverged or is ahead; update skipped to protect its commits.' >&2
        exit 1
    fi
    if [[ "$current_branch" != "$branch" ]]; then
        git -C "$repo" switch --no-recurse-submodules --no-overwrite-ignore -- "$branch"
    fi
    git -C "$repo" -c submodule.recurse=false merge --ff-only --no-overwrite-ignore "$revision"
else
    git -C "$repo" switch --no-recurse-submodules --no-overwrite-ignore -c "$branch" "$revision"
    if git -C "$repo" show-ref --verify --quiet "refs/remotes/origin/$branch"; then
        git -C "$repo" branch --set-upstream-to="origin/$branch" "$branch"
    fi
fi
echo "[jetson] Checkout ready: $branch at ${revision:0:12}."
