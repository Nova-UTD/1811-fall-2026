#!/usr/bin/env bash
# Sent over SSH by the Karbon, so it also works with an older Jetson checkout.
set -euo pipefail

if (( $# != 3 && $# != 4 )) || { (( $# == 4 )) && [[ "$4" != --check ]]; }; then
    echo '[jetson] [ERROR] Invalid sync arguments: expected repository path, branch, commit, and optional --check.' >&2
    exit 2
fi
repo=$1 branch=$2 revision=$3
check_only=false
[[ ${4:-} != --check ]] || check_only=true
case "$repo" in
    '~') repo=$HOME ;;
    '~/'*) repo="$HOME/${repo#\~/}" ;;
esac

if [[ ! "$revision" =~ ^([[:xdigit:]]{40}|[[:xdigit:]]{64})$ ]] ||
   ! git check-ref-format --branch "$branch" >/dev/null 2>&1; then
    echo '[jetson] [ERROR] Karbon branch or commit is missing; commit your local Jetson changes on the Karbon and retry.' >&2
    exit 1
fi
if ! git -C "$repo" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "[jetson] [ERROR] No Git checkout found at $repo; complete the one-time Jetson setup first." >&2
    exit 1
fi
if ! changes=$(git -C "$repo" status --porcelain --untracked-files=normal); then
    echo '[jetson] [ERROR] Could not inspect the Jetson Git working tree.' >&2
    exit 1
fi
if [[ -n "$changes" ]]; then
    echo '[jetson] [ERROR] Jetson checkout has local changes; skipping update to avoid overwriting them.' >&2
    exit 1
fi

current_branch=$(git -C "$repo" symbolic-ref --quiet --short HEAD || true)
current_revision=$(git -C "$repo" rev-parse --verify HEAD)
if [[ "$current_branch" == "$branch" && "$current_revision" == "$revision" ]]; then
    echo "[jetson] Jetson checkout already matches Karbon ($branch at ${revision:0:12})."
    exit 0
fi

if [[ "$check_only" == false ]]; then
    if ! command -v pgrep >/dev/null 2>&1; then
        echo '[jetson] [ERROR] Cannot check for running Jetson camera processes because pgrep is not installed.' >&2
        exit 1
    fi
    # A check can run while cameras are active; an actual checkout cannot.
    if pgrep -f '[r]os2 launch (jetson_bringup zed_cameras[.]launch[.]py|camera_panorama panorama[.]launch[.]py|lidar_perception (corner_detector|lidar_camera_registration)[.]launch[.]py)' >/dev/null; then
        echo '[jetson] [ERROR] Jetson cameras or registration nodes are already running; stop them before updating the checkout.' >&2
        exit 1
    fi
fi

if ! git -C "$repo" cat-file -e "$revision^{commit}" 2>/dev/null; then
    echo "[jetson] Fetching latest $branch commits from origin..."
    if ! timeout 30s env GIT_TERMINAL_PROMPT=0 \
        GIT_SSH_COMMAND='ssh -o BatchMode=yes -o ConnectTimeout=5' \
        git -C "$repo" fetch --no-tags --no-recurse-submodules origin "$branch" </dev/null; then
        echo '[jetson] [ERROR] Could not fetch from origin on the Jetson; check network connectivity and Git credentials.' >&2
        exit 1
    fi
fi
if ! git -C "$repo" cat-file -e "$revision^{commit}" 2>/dev/null; then
    echo '[jetson] [ERROR] Karbon commit is not available on origin yet; push the Karbon branch first.' >&2
    exit 1
fi

if git -C "$repo" show-ref --verify --quiet "refs/heads/$branch"; then
    if ! git -C "$repo" merge-base --is-ancestor "refs/heads/$branch" "$revision"; then
        echo '[jetson] [ERROR] Jetson branch has diverged or is ahead of the Karbon; skipping update to protect local Jetson commits.' >&2
        exit 1
    fi
    if [[ "$check_only" == true ]]; then
        echo '[jetson] Jetson checkout verified and ready to update.'
        exit 0
    fi
    if [[ "$current_branch" != "$branch" ]]; then
        git -C "$repo" switch --no-recurse-submodules --no-overwrite-ignore -- "$branch"
    fi
    git -C "$repo" -c submodule.recurse=false merge --ff-only --no-overwrite-ignore "$revision"
else
    if [[ "$check_only" == true ]]; then
        echo '[jetson] Checkout is ready to update.'
        exit 0
    fi
    git -C "$repo" switch --no-recurse-submodules --no-overwrite-ignore -c "$branch" "$revision"
    if git -C "$repo" show-ref --verify --quiet "refs/remotes/origin/$branch"; then
        git -C "$repo" branch --set-upstream-to="origin/$branch" "$branch"
    fi
fi
echo "[jetson] Jetson checkout updated to $branch at ${revision:0:12}."
