#!/usr/bin/env bash
# Kill local training screen session.
# Usage: kill.sh <branch-sanitized>

set -euo pipefail

[ $# -lt 1 ] && { echo "Usage: kill.sh <branch-sanitized>" >&2; exit 1; }

BRANCH_SANITIZED="$1"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR=$(grep "^- Remote dir:" "$SCRIPT_DIR/host.md" | sed 's/^- Remote dir:[[:space:]]*//')
PROJECT_NAME="$(basename "$REPO_DIR")"
SCREEN_NAME="$PROJECT_NAME-$BRANCH_SANITIZED"

if ! screen -ls | grep -q "$SCREEN_NAME"; then
    echo "No screen session '$SCREEN_NAME' found — nothing to kill."
    exit 0
fi

echo "Sending Ctrl-C to $SCREEN_NAME..."
screen -S "$SCREEN_NAME" -X stuff $'\003' || true
sleep 2

if screen -ls | grep -q "$SCREEN_NAME"; then
    echo "Screen still exists, sending quit..."
    screen -S "$SCREEN_NAME" -X quit 2>/dev/null || true
fi

echo "Training killed."
