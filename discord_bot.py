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
    !video <branch>  — post latest evaluation video
    !help            — list commands

Notifications (sent automatically by notify.sh):
    Training started, monitor updates, training killed, blockers.

Free-text messages (not starting with !):
    Interpreted as human feedback for the active training session.
    Tags are extracted via Claude API and stored with feedback.py.

Secrets: read from .discord_secrets in the repo root.
    BOT_TOKEN=...
    CHANNEL_ID=...
"""

import asyncio
import json
import os
import re
import subprocess
from pathlib import Path

import anthropic
import discord

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).parent
SECRETS_FILE = REPO_ROOT / ".discord_secrets"
SESSION_DIR = REPO_ROOT / "logs" / "sessions"
RL_CONFIG = REPO_ROOT / ".claude" / "rl-training" / "config.md"
TASK_CONFIG = REPO_ROOT / ".claude" / "rl-training" / "tasks" / "locomotion" / "monitor_config.md"
FEEDBACK_SCRIPT = REPO_ROOT / ".claude" / "rl-training" / "scripts" / "feedback.py"


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
    monitors = sorted(run_dir.glob("monitor_*.md")) if run_dir.exists() else []
    if not monitors:
        return f"No monitor files yet for run {run_num:03d}"
    latest = monitors[-1].read_text()
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


def find_latest_video(branch_sanitized: str) -> Path | None:
    """Return the most recent eval_video.mp4 path for this branch, or None."""
    session_dir = SESSION_DIR / branch_sanitized
    if not session_dir.exists():
        return None
    state_file = session_dir / "session_state.json"
    if not state_file.exists():
        return None
    state = json.loads(state_file.read_text())
    run_num = state.get("current_run", 1)
    # Search from current run back to run 1
    for r in range(run_num, 0, -1):
        video = session_dir / f"run_{r:03d}" / "eval_video.mp4"
        if video.exists():
            return video
    return None


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


def _load_feedback_tags() -> dict[str, str]:
    """Parse human feedback tags from monitor_config.md."""
    if not TASK_CONFIG.exists():
        return {}
    tags = {}
    in_section = False
    for line in TASK_CONFIG.read_text().splitlines():
        if "## Human Feedback Tags" in line:
            in_section = True
            continue
        if in_section and line.startswith("##"):
            break
        if in_section:
            m = re.match(r"^- (\w+):\s*(.+)$", line)
            if m:
                tags[m.group(1)] = m.group(2).strip()
    return tags


def classify_feedback(text: str, tags: dict[str, str]) -> dict:
    """Use Claude Haiku to extract feedback tags from free-text."""
    tag_list = "\n".join(f"- {name}: {desc}" for name, desc in tags.items())
    try:
        ai = anthropic.Anthropic()
        resp = ai.messages.create(
            model="claude-haiku-4-5-20251001",
            max_tokens=256,
            messages=[{
                "role": "user",
                "content": (
                    "You classify observer feedback on a robot walking trial.\n\n"
                    f"Available tags:\n{tag_list}\n\n"
                    f'Observer message: "{text}"\n\n'
                    "Return a JSON object with:\n"
                    '- "tags": list of matching tag names (empty list if none)\n'
                    '- "notes": the original message verbatim\n'
                    "Only return the JSON object, no commentary."
                ),
            }],
        )
        return json.loads(resp.content[0].text.strip())
    except Exception as e:
        return {"tags": [], "notes": text, "error": str(e)}


def persist_feedback(branch_sanitized: str, run_num: int, tags: list[str], notes: str) -> str:
    session_dir = SESSION_DIR / branch_sanitized
    result = subprocess.run(
        [
            "uv", "run", str(FEEDBACK_SCRIPT),
            str(session_dir),
            "--run", str(run_num),
            "--tags", ",".join(tags),
            "--notes", notes,
            "--task-config", str(TASK_CONFIG),
        ],
        capture_output=True, text=True, cwd=str(REPO_ROOT), timeout=30,
    )
    return (result.stdout + result.stderr).strip()


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

    if message.channel.id != channel_id or message.author == client.user:
        return

    content = message.content.strip()

    # ------------------------------------------------------------------ #
    # Commands (start with !)                                              #
    # ------------------------------------------------------------------ #
    if content.startswith("!"):
        parts = content.split()
        cmd = parts[0].lower()

        if cmd == "!help":
            help_text = (
                "**RL Training Bot Commands**\n"
                "`!status` — show all training sessions\n"
                "`!kill <branch>` — kill training for a branch\n"
                "`!logs <branch>` — latest monitor report\n"
                "`!metrics <branch>` — latest quality scores\n"
                "`!video <branch>` — post latest evaluation video\n"
                "`!help` — this message\n\n"
                "**Tip:** send any free-text message and I'll interpret it as "
                "human feedback for the active training session."
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

        elif cmd == "!video":
            if len(parts) < 2:
                await message.channel.send("Usage: `!video <branch>`")
                return
            branch_sanitized = parts[1].replace("/", "--")
            video_path = await asyncio.get_event_loop().run_in_executor(
                None, find_latest_video, branch_sanitized
            )
            if video_path is None:
                await message.channel.send(
                    f"No eval video found for `{branch_sanitized}`. "
                    "Videos are generated during policy evaluation (each monitor tick)."
                )
            else:
                await message.channel.send(
                    f"Latest eval video for `{branch_sanitized}` (run {video_path.parent.name}):",
                    file=discord.File(str(video_path)),
                )

        return  # handled as command

    # ------------------------------------------------------------------ #
    # Free-text → human feedback                                           #
    # ------------------------------------------------------------------ #
    sessions = get_sessions()
    active = [s for s in sessions if s.get("phase") in ("MONITOR", "ITERATE", "LAUNCH")]
    if not active:
        return  # nothing to attach feedback to

    if len(active) > 1:
        branches = ", ".join(f"`{s.get('branch', '?')}`" for s in active)
        await message.channel.send(
            f"Multiple active sessions ({branches}). "
            "Use `!feedback <branch> <message>` to target a specific one.\n"
            "_(For now I'll attach your feedback to the most recently started session.)_"
        )
        # Fall through: attach to last active session by default
        session = active[-1]
    else:
        session = active[0]

    branch = session.get("branch", "?")
    branch_sanitized = branch.replace("/", "--")
    run_num = session.get("current_run", 1)

    await message.add_reaction("⏳")

    tags_map = _load_feedback_tags()
    classified = await asyncio.get_event_loop().run_in_executor(
        None, classify_feedback, content, tags_map
    )

    tags = classified.get("tags", [])
    notes = classified.get("notes", content)
    err = classified.get("error")

    if err:
        await message.channel.send(f"⚠️ Classification failed ({err}) — storing as raw note.")

    persist_result = await asyncio.get_event_loop().run_in_executor(
        None, persist_feedback, branch_sanitized, run_num, tags, notes
    )

    tag_str = ", ".join(f"`{t}`" for t in tags) if tags else "_(no matching tags)_"
    reply = (
        f"Feedback stored for `{branch}` run {run_num}.\n"
        f"Tags: {tag_str}\n"
        f"Note: {notes[:200]}"
    )
    await message.remove_reaction("⏳", client.user)
    await message.channel.send(reply)


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
