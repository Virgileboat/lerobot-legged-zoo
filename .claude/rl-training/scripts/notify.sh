#!/usr/bin/env bash
# Send a notification to Discord via bot token (direct API POST).
# Usage: notify.sh "<message>" [--branch <name>] [--file <path>]
#
# --branch: prepends [<branch>] to the message
# --file: attaches a file (e.g. video)
#
# Reads BOT_TOKEN and CHANNEL_ID from .discord_secrets in the project root.
# IMPORTANT: pass actual newlines in the message, not literal \n.
# Use printf or $'...' syntax to compose multi-line messages.

set -euo pipefail

if ! command -v jq >/dev/null 2>&1; then
    echo "jq not found — installing..." >&2
    if command -v apt-get >/dev/null 2>&1; then sudo apt-get install -y jq
    elif command -v brew >/dev/null 2>&1; then brew install jq
    else echo "ERROR: Cannot auto-install jq" >&2; exit 1; fi
fi

[ $# -lt 1 ] && { echo "Usage: notify.sh \"<message>\" [--branch <name>] [--file <path>]" >&2; exit 1; }

MESSAGE="$1"
shift
FILE=""
BRANCH=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --file)   [ $# -ge 2 ] || { echo "ERROR: --file requires a value" >&2; exit 1; }; FILE="$2"; shift 2 ;;
        --branch) [ $# -ge 2 ] || { echo "ERROR: --branch requires a value" >&2; exit 1; }; BRANCH="$2"; shift 2 ;;
        *) echo "WARNING: Unknown argument: $1" >&2; shift ;;
    esac
done

[ -n "$BRANCH" ] && MESSAGE="[$BRANCH] $MESSAGE"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
SECRETS="$PROJECT_ROOT/.discord_secrets"

if [ ! -f "$SECRETS" ]; then
    echo "WARNING: .discord_secrets not found — skipping notification" >&2
    exit 0
fi

BOT_TOKEN=$(grep "^BOT_TOKEN=" "$SECRETS" | head -1 | cut -d= -f2-)
CHANNEL_ID=$(grep "^CHANNEL_ID=" "$SECRETS" | head -1 | cut -d= -f2-)

if [ -z "$BOT_TOKEN" ] || [ "$BOT_TOKEN" = "PASTE_YOUR_BOT_TOKEN_HERE" ]; then
    echo "WARNING: BOT_TOKEN not set in .discord_secrets — skipping" >&2
    exit 0
fi
if [ -z "$CHANNEL_ID" ] || [ "$CHANNEL_ID" = "PASTE_YOUR_CHANNEL_ID_HERE" ]; then
    echo "WARNING: CHANNEL_ID not set in .discord_secrets — skipping" >&2
    exit 0
fi

API_URL="https://discord.com/api/v10/channels/$CHANNEL_ID/messages"
MESSAGE="${MESSAGE:0:2000}"

if [ -n "$FILE" ] && [ -f "$FILE" ]; then
    curl -s \
        -H "Authorization: Bot $BOT_TOKEN" \
        -F "payload_json={\"content\":$(printf '%s' "$MESSAGE" | jq -Rs .)}" \
        -F "file=@$FILE" \
        "$API_URL" > /dev/null
else
    curl -s \
        -H "Authorization: Bot $BOT_TOKEN" \
        -H "Content-Type: application/json" \
        -d "{\"content\":$(printf '%s' "$MESSAGE" | jq -Rs .)}" \
        "$API_URL" > /dev/null
fi
