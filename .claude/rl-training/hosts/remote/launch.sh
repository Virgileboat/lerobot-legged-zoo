#!/usr/bin/env bash
# Launch training locally using git worktree + screen.
# Usage: launch.sh <branch> <branch-sanitized> <train-command> [<deps-command>]
# Exit 0: training started. Exit 1: GPU unavailable.

set -euo pipefail

[ $# -lt 3 ] && { echo "Usage: launch.sh <branch> <branch-sanitized> <train-command> [<deps-command>]" >&2; exit 1; }

BRANCH="$1"
BRANCH_SANITIZED="$2"
TRAIN_CMD="$3"
DEPS_CMD="${4:-}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR=$(grep "^- Remote dir:" "$SCRIPT_DIR/host.md" | sed 's/^- Remote dir:[[:space:]]*//')
GPU_CHECK=$(grep "^- GPU check:" "$SCRIPT_DIR/host.md" | sed 's/^- GPU check:[[:space:]]*//')
GPU_THRESHOLD=$(grep "^- GPU threshold:" "$SCRIPT_DIR/host.md" | sed 's/^- GPU threshold:[[:space:]]*//')
HOST_DEPS=$(grep "^- Dependencies:" "$SCRIPT_DIR/host.md" | sed 's/^- Dependencies:[[:space:]]*//')

[ -z "$REPO_DIR" ] && { echo "ERROR: Remote dir not found in host.md" >&2; exit 1; }

PROJECT_NAME="$(basename "$REPO_DIR")"
WORKTREE_DIR="${REPO_DIR%/*}/$PROJECT_NAME-wt-$BRANCH_SANITIZED"
SCREEN_NAME="$PROJECT_NAME-$BRANCH_SANITIZED"

if [ -n "$GPU_CHECK" ]; then
    echo "=== Checking GPU usage ==="
    GPU_MAX=$($GPU_CHECK | awk '{if($1+0 > max) max=$1+0} END{print max+0}' 2>/dev/null || echo "0")
    echo "GPU max utilization: ${GPU_MAX}%"
    if [ "${GPU_MAX:-0}" -gt "${GPU_THRESHOLD:-80}" ]; then
        echo "ERROR: GPU utilization ${GPU_MAX}% > threshold ${GPU_THRESHOLD}%" >&2
        exit 1
    fi
fi

echo "=== Setting up worktree ==="
cd "$REPO_DIR"
git fetch origin 2>/dev/null || echo "Warning: could not fetch from remote, using local branch state"

# If main repo is already on this branch, use it directly (no separate worktree needed)
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "")
if [ "$CURRENT_BRANCH" = "$BRANCH" ]; then
    echo "Main repo is already on $BRANCH — using repo dir directly"
    WORKTREE_DIR="$REPO_DIR"
else
    git worktree add "$WORKTREE_DIR" "$BRANCH" 2>/dev/null || (cd "$WORKTREE_DIR" && git checkout "$BRANCH" && git pull origin "$BRANCH" 2>/dev/null || echo "Warning: could not pull, using existing worktree state")
fi

ACTUAL_DEPS="${DEPS_CMD:-$HOST_DEPS}"
if [ -n "$ACTUAL_DEPS" ]; then
    echo "=== Syncing dependencies ==="
    (cd "$WORKTREE_DIR" && $ACTUAL_DEPS)
fi

FULL_TRAIN_CMD="$TRAIN_CMD"

echo "=== Launching training in screen $SCREEN_NAME ==="
screen -ls | grep -q "$SCREEN_NAME" && screen -S "$SCREEN_NAME" -X stuff $'\003' || screen -dmS "$SCREEN_NAME"
sleep 1

screen -S "$SCREEN_NAME" -X stuff "cd \"$WORKTREE_DIR\" && $FULL_TRAIN_CMD"$'\n'

echo "=== Training launched ==="
echo "Worktree: $WORKTREE_DIR"
echo "Screen:   $SCREEN_NAME"
echo "Monitor:  screen -r $SCREEN_NAME"
