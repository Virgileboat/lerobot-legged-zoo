#!/usr/bin/env bash
# Initialize or reset a training session directory.
# Usage: .claude/rl-training/scripts/init_session.sh "<goal>" "<branch>"
#
# Creates logs/sessions/<branch-sanitized>/ with session_state.json.
# Branch sanitization: / → --

set -euo pipefail

if ! command -v jq >/dev/null 2>&1; then
    echo "jq not found — installing..." >&2
    if command -v brew >/dev/null 2>&1; then brew install jq
    elif command -v apt-get >/dev/null 2>&1; then sudo apt-get install -y jq
    elif command -v yum >/dev/null 2>&1; then sudo yum install -y jq
    elif command -v pacman >/dev/null 2>&1; then sudo pacman -S --noconfirm jq
    else echo "ERROR: Cannot auto-install jq — install it manually" >&2; exit 1; fi
fi
[ $# -lt 2 ] && { echo "Usage: init_session.sh \"<goal>\" \"<branch>\"" >&2; exit 1; }

GOAL="$1"
BRANCH="$2"
BRANCH_SANITIZED="${BRANCH//\//--}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
SESSION_DIR="$PROJECT_ROOT/logs/sessions/$BRANCH_SANITIZED"

if [ -f "$SESSION_DIR/session_state.json" ]; then
    BACKUP="$SESSION_DIR/session_state.$(date +%Y%m%d_%H%M%S).json"
    cp "$SESSION_DIR/session_state.json" "$BACKUP"
    echo "Previous state backed up to $BACKUP"
fi

NEXT_RUN=1
if [ -d "$SESSION_DIR" ]; then
    LAST_RUN=$(ls -d "$SESSION_DIR"/run_* 2>/dev/null | sort -V | tail -1 | grep -oE '[0-9]+$' || echo "0")
    NEXT_RUN=$((10#$LAST_RUN + 1))
fi

RUN_DIR=$(printf "run_%03d" "$NEXT_RUN")
mkdir -p "$SESSION_DIR/$RUN_DIR"

jq -n \
  --arg goal "$GOAL" \
  --arg branch "$BRANCH" \
  --arg branch_sanitized "$BRANCH_SANITIZED" \
  --argjson run "$NEXT_RUN" \
  '{goal: $goal, branch: $branch, branch_sanitized: $branch_sanitized, current_run: $run, wandb_run_path: "", host: "", phase: "LAUNCH", monitor_count: 0, consecutive_bad: 0, iterations: []}' \
  > "$SESSION_DIR/session_state.json"

echo "Session initialized at $SESSION_DIR"
echo "Run directory: $SESSION_DIR/$RUN_DIR"
echo "State: $SESSION_DIR/session_state.json"
