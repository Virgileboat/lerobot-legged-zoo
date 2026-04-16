#!/usr/bin/env python3
"""Discord bot for RL training control.

Runs as a persistent process in a screen session on the training host.
Sends training notifications and handles commands from Discord.

Usage:
    screen -dmS discord-bot uv run discord_bot.py

Commands (in the configured Discord channel):
    !status          — show all active training sessions
    !kill <branch>   — kill training for a branch
    !logs <branch>   — show latest monitor report
    !metrics <branch>— show latest quality scores
    !help            — list commands

Notifications (sent automatically by notify.sh):
    Training started, monitor updates, training killed, blockers.

Secrets: read from .discord_secrets in the repo root.
    BOT_TOKEN=...
    CHANNEL_ID=...
"""

import asyncio
import json
import os
import subprocess
from pathlib import Path

import discord

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).parent
SECRETS_FILE = REPO_ROOT / ".discord_secrets"
SESSION_DIR = REPO_ROOT / "logs" / "sessions"
RL_CONFIG = REPO_ROOT / ".claude" / "rl-training" / "config.md"


def load_secrets() -> dict:
    if not SECRETS_FILE.exists():
        raise FileNotFoundError(f"Missing {SECRETS_FILE} — create it with BOT_TOKEN and CHANNEL_ID")
    secrets = {}
    for line in SECRETS_FILE.read_text().splitlines():
        line = line.strip()
        if "=" in line and not line.startswith("#"):
            key, _, val = line.partition("=")
            secrets[key.strip()] = val.strip()
    return secrets


# ---------------------------------------------------------------------------
# Bot
# ---------------------------------------------------------------------------

intents = discord.Intents.default()
intents.message_content = True
client = discord.Client(intents=intents)


def get_sessions() -> list[dict]:
    """Read all session_state.json files."""
    sessions = []
    if not SESSION_DIR.exists():
        return sessions
    for state_file in sorted(SESSION_DIR.glob("*/session_state.json")):
        try:
            state = json.loads(state_file.read_text())
            sessions.append(state)
        except Exception:
            pass
    return sessions


def format_status() -> str:
    sessions = get_sessions()
    if not sessions:
        return "No training sessions found."
    lines = ["**Training Sessions**"]
    for s in sessions:
        branch = s.get("branch", "?")
        phase = s.get("phase", "?")
        run = s.get("current_run", "?")
        host = s.get("host", "local")
        goal = s.get("goal", "")
        wandb = s.get("wandb_run_path", "")
        status_icon = {"MONITOR": "🟢", "ITERATE": "🔄", "FINISHED": "✅", "PAUSED": "⏸️", "LAUNCH": "🚀"}.get(phase, "⚪")
        lines.append(f"{status_icon} `{branch}` — phase: **{phase}** | run: {run} | host: {host}")
        if goal:
            lines.append(f"   Goal: {goal}")
        if wandb:
            lines.append(f"   WandB: `{wandb}`")
    return "\n".join(lines)


def get_latest_monitor(branch_sanitized: str) -> str:
    session_dir = SESSION_DIR / branch_sanitized
    if not session_dir.exists():
        return f"No session found for `{branch_sanitized}`"
    state_file = session_dir / "session_state.json"
    if not state_file.exists():
        return "No session_state.json"
    state = json.loads(state_file.read_text())
    run_num = state.get("current_run", 1) - 1  # last completed run
    if run_num < 1:
        run_num = 1
    run_dir = session_dir / f"run_{run_num:03d}"
    # Find latest monitor file
    monitors = sorted(run_dir.glob("monitor_*.md")) if run_dir.exists() else []
    if not monitors:
        return f"No monitor files yet for run {run_num:03d}"
    latest = monitors[-1].read_text()
    # Trim to Discord 2000 char limit (strip RAW_METRICS block)
    if "<!-- RAW_METRICS:" in latest:
        latest = latest[:latest.index("<!-- RAW_METRICS:")]
    return latest.strip()[:1900]


def get_latest_metrics(branch_sanitized: str) -> str:
    session_dir = SESSION_DIR / branch_sanitized
    if not session_dir.exists():
        return f"No session found for `{branch_sanitized}`"
    state = json.loads((session_dir / "session_state.json").read_text())
    run_num = state.get("current_run", 1)
    run_dir = session_dir / f"run_{run_num:03d}"
    derived_files = sorted(run_dir.glob("derived_metrics_*.json")) if run_dir.exists() else []
    if not derived_files:
        return f"No quality metrics yet for run {run_num:03d}"
    data = json.loads(derived_files[-1].read_text())
    lines = [f"**Quality Metrics — run {run_num:03d} (monitor {derived_files[-1].stem.split('_')[-1]})**"]
    for k, v in data.items():
        if k == "missing":
            continue
        if isinstance(v, float):
            lines.append(f"- **{k}**: {v:.3f}")
        elif isinstance(v, list):
            lines.append(f"- **{k}**: {', '.join(v)}")
        else:
            lines.append(f"- **{k}**: {v}")
    return "\n".join(lines)


def kill_training(branch_sanitized: str) -> str:
    kill_script = REPO_ROOT / ".claude" / "rl-training" / "hosts" / "remote" / "kill.sh"
    if not kill_script.exists():
        return "kill.sh not found"
    result = subprocess.run(
        ["bash", str(kill_script), branch_sanitized],
        capture_output=True, text=True, timeout=30
    )
    output = (result.stdout + result.stderr).strip()
    return output[:1500] if output else ("Killed." if result.returncode == 0 else "Kill failed (no output)")


# ---------------------------------------------------------------------------
# Event handlers
# ---------------------------------------------------------------------------

@client.event
async def on_ready():
    print(f"Bot ready: {client.user} (id={client.user.id})")


@client.event
async def on_message(message: discord.Message):
    secrets = client._secrets
    channel_id = int(secrets.get("CHANNEL_ID", "0"))

    # Only respond in the configured channel, ignore own messages
    if message.channel.id != channel_id or message.author == client.user:
        return

    content = message.content.strip()
    if not content.startswith("!"):
        return

    parts = content.split()
    cmd = parts[0].lower()

    if cmd == "!help":
        help_text = (
            "**RL Training Bot Commands**\n"
            "`!status` — show all training sessions\n"
            "`!kill <branch>` — kill training for a branch\n"
            "`!logs <branch>` — latest monitor report\n"
            "`!metrics <branch>` — latest quality scores\n"
            "`!help` — this message"
        )
        await message.channel.send(help_text)

    elif cmd == "!status":
        await message.channel.send(format_status())

    elif cmd == "!kill":
        if len(parts) < 2:
            await message.channel.send("Usage: `!kill <branch>` — e.g. `!kill claude--symmetric-walk`")
            return
        branch_sanitized = parts[1].replace("/", "--")
        await message.channel.send(f"Killing training for `{branch_sanitized}`...")
        result = await asyncio.get_event_loop().run_in_executor(None, kill_training, branch_sanitized)
        await message.channel.send(f"```\n{result}\n```")

    elif cmd == "!logs":
        if len(parts) < 2:
            await message.channel.send("Usage: `!logs <branch>`")
            return
        branch_sanitized = parts[1].replace("/", "--")
        logs = await asyncio.get_event_loop().run_in_executor(None, get_latest_monitor, branch_sanitized)
        await message.channel.send(f"```\n{logs}\n```")

    elif cmd == "!metrics":
        if len(parts) < 2:
            await message.channel.send("Usage: `!metrics <branch>`")
            return
        branch_sanitized = parts[1].replace("/", "--")
        metrics = await asyncio.get_event_loop().run_in_executor(None, get_latest_metrics, branch_sanitized)
        await message.channel.send(metrics)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    secrets = load_secrets()
    token = secrets.get("BOT_TOKEN", "")
    channel_id = secrets.get("CHANNEL_ID", "")

    if not token or token == "PASTE_YOUR_BOT_TOKEN_HERE":
        raise ValueError("BOT_TOKEN not set in .discord_secrets")
    if not channel_id or channel_id == "PASTE_YOUR_CHANNEL_ID_HERE":
        raise ValueError("CHANNEL_ID not set in .discord_secrets")

    client._secrets = secrets
    print(f"Starting bot... channel={channel_id}")
    client.run(token)


if __name__ == "__main__":
    main()
