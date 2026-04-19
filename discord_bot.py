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


CLAUDE_BIN = "/home/vbatto/.local/bin/claude"
CLAUDE_MODEL = "claude-haiku-4-5-20251001"
ITERATE_CLAUDE_MODEL = "claude-sonnet-4-6"

_QUESTION_STARTERS = (
    "what ", "why ", "how ", "when ", "where ", "who ", "which ",
    "can ", "could ", "would ", "should ", "will ", "is ", "are ",
    "was ", "were ", "do ", "does ", "did ", "has ", "have ", "had ",
    "tell me", "explain", "describe", "show me",
)

_MODIFICATION_VERBS = (
    "increase", "decrease", "reduce", "lower", "raise", "bump", "boost",
    "add", "remove", "set", "change", "modify", "update", "adjust", "tune",
    "try ", "use ", "switch", "enable", "disable", "apply",
    "kill and", "stop and", "restart with",
)

# Keywords anywhere in the message that signal a code/config change request
_MODIFICATION_KEYWORDS = (
    "debug", "fix", "rewrite", "refactor", "implement", "rework",
    "reward", "curriculum", "weight", "penalty", "observation", "action",
    "symmetr", "metric", "trajectory", "foot", "joint", "hip", "knee", "ankle",
)


def is_question(text: str) -> bool:
    """Return True if the message looks like a question rather than feedback."""
    t = text.lower().strip()
    return t.endswith("?") or any(t.startswith(q) for q in _QUESTION_STARTERS)


def is_modification_request(text: str) -> bool:
    """Return True if the message looks like a training modification request."""
    t = text.lower().strip()
    # Starts with an action verb
    if any(t.startswith(v) for v in _MODIFICATION_VERBS):
        return True
    # Contains actionable keywords AND looks instructional (has a verb somewhere)
    has_keyword = any(k in t for k in _MODIFICATION_KEYWORDS)
    has_action = any(v.strip() in t for v in _MODIFICATION_VERBS)
    return has_keyword and has_action


def _load_training_context(branch_sanitized: str) -> str:
    """Build a concise training context string for the QA system prompt."""
    session_dir = SESSION_DIR / branch_sanitized
    if not session_dir.exists():
        return "No active training session found."
    state_file = session_dir / "session_state.json"
    if not state_file.exists():
        return "No session state found."
    state = json.loads(state_file.read_text())
    run_num = state.get("current_run", 1)
    run_dir = session_dir / f"run_{run_num:03d}"
    parts = [
        f"Branch: {state.get('branch', '?')}",
        f"Goal: {state.get('goal', 'unknown')}",
        f"Phase: {state.get('phase', '?')} | Run: {run_num} | Host: {state.get('host', '?')}",
        f"WandB: {state.get('wandb_run_path', 'N/A')}",
    ]
    if run_dir.exists():
        monitors = sorted(run_dir.glob("monitor_*.md"))
        if monitors:
            latest = monitors[-1].read_text()
            if "<!-- RAW_METRICS:" in latest:
                latest = latest[: latest.index("<!-- RAW_METRICS:")]
            parts.append(f"\nLatest monitor:\n{latest.strip()[:1200]}")
        eval_report = run_dir / "eval_report.md"
        if eval_report.exists():
            report = eval_report.read_text()
            if "<!-- RAW_METRICS:" in report:
                report = report[: report.index("<!-- RAW_METRICS:")]
            parts.append(f"\nBehavioral evaluation:\n{report.strip()[:800]}")
        derived = sorted(run_dir.glob("derived_metrics_*.json"))
        if derived:
            data = json.loads(derived[-1].read_text())
            summary = {k: v for k, v in data.items() if k != "missing"}
            parts.append(f"\nQuality metrics: {json.dumps(summary)}")
    return "\n".join(parts)


def answer_question(question: str, context: str) -> str:
    """Use Claude to answer a question about the current training run."""
    try:
        prompt = (
            "You are an RL training assistant for a legged humanoid robot (12-DOF biped, no arms). "
            "You help interpret training metrics and behavioral evaluations. "
            "Be concise and precise. When referencing numbers use the context provided.\n\n"
            f"Current training context:\n{context}\n\n"
            f"Question: {question}"
        )
        return _claude(prompt, max_tokens=512)
    except Exception as e:
        return f"⚠️ Could not answer: {e}"


def parse_modification_request(text: str, context: str) -> dict:
    """Use Claude to parse a training modification request into a structured intent."""
    try:
        prompt = (
            "You are an RL training assistant for a legged humanoid robot. "
            "Parse training modification requests into structured JSON.\n\n"
            f"Current training context:\n{context}\n\n"
            f'Modification request: "{text}"\n\n'
            "Return a JSON object with:\n"
            '- "summary": one-line description of the requested change\n'
            '- "parameters": list of affected training parameters (reward weights, curriculum, etc.)\n'
            '- "urgency": "immediate" (kill current run now) or "next_iterate" (apply when training ends naturally)\n'
            '- "notes": full user request verbatim\n'
            "Only return the JSON object, no commentary."
        )
        return json.loads(_claude(prompt, max_tokens=256))
    except Exception as e:
        return {
            "summary": text[:100],
            "parameters": [],
            "urgency": "next_iterate",
            "notes": text,
            "error": str(e),
        }


def queue_modification(branch_sanitized: str, run_num: int, parsed: dict) -> None:
    """Write a pending modification request to the session directory."""
    session_dir = SESSION_DIR / branch_sanitized
    queue_file = session_dir / "pending_modification.json"
    entry = {
        "run": run_num,
        "summary": parsed.get("summary", ""),
        "parameters": parsed.get("parameters", []),
        "urgency": parsed.get("urgency", "next_iterate"),
        "notes": parsed.get("notes", ""),
    }
    # Append to existing queue or start new one
    queue = []
    if queue_file.exists():
        try:
            queue = json.loads(queue_file.read_text())
        except Exception:
            pass
    queue.append(entry)
    queue_file.write_text(json.dumps(queue, indent=2))


def _claude(prompt: str, max_tokens: int = 512) -> str:
    """Run a one-shot Claude Code query and return the text response."""
    result = subprocess.run(
        [CLAUDE_BIN, "--print", "--model", CLAUDE_MODEL, prompt],
        capture_output=True, text=True, timeout=60, cwd=str(REPO_ROOT),
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "claude exited non-zero")
    return result.stdout.strip()


def classify_feedback(text: str, tags: dict[str, str]) -> dict:
    """Use Claude to extract feedback tags from free-text."""
    tag_list = "\n".join(f"- {name}: {desc}" for name, desc in tags.items())
    try:
        prompt = (
            "You classify observer feedback on a robot walking trial.\n\n"
            f"Available tags:\n{tag_list}\n\n"
            f'Observer message: "{text}"\n\n'
            "Return a JSON object with:\n"
            '- "tags": list of matching tag names (empty list if none)\n'
            '- "notes": the original message verbatim\n'
            "Only return the JSON object, no commentary."
        )
        return json.loads(_claude(prompt, max_tokens=256))
    except Exception as e:
        return {"tags": [], "notes": text, "error": str(e)}


def apply_modification_now(branch_sanitized: str) -> str:
    """Kill training immediately and set phase=ITERATE to apply pending modification."""
    session_dir = SESSION_DIR / branch_sanitized
    queue_file = session_dir / "pending_modification.json"

    if not queue_file.exists():
        # Fall back to latest human_feedback for current run stored in session_state
        state_file = session_dir / "session_state.json"
        if state_file.exists():
            try:
                state = json.loads(state_file.read_text())
                current_run = state.get("current_run", 1)
                for entry in reversed(state.get("iterations", [])):
                    if entry.get("run") == current_run and entry.get("human_feedback"):
                        notes = entry["human_feedback"].get("notes", "").strip()
                        if notes:
                            fallback_mod = [{
                                "run": current_run,
                                "summary": notes[:100],
                                "parameters": [],
                                "urgency": "immediate",
                                "notes": notes,
                            }]
                            queue_file.write_text(json.dumps(fallback_mod, indent=2))
                            break
            except Exception:
                pass
        if not queue_file.exists():
            return "No pending modification found. Queue one first with a free-text message."

    queue = json.loads(queue_file.read_text())
    if not queue:
        return "Pending modification file is empty."
    mod = queue[-1]

    # Kill training
    kill_script = REPO_ROOT / ".claude" / "rl-training" / "hosts" / "remote" / "kill.sh"
    kill_result = subprocess.run(
        ["bash", str(kill_script), branch_sanitized],
        capture_output=True, text=True, timeout=30,
    )

    # Consume the modification
    consumed_file = session_dir / "consumed_modification.json"
    queue_file.rename(consumed_file)

    # Read current state
    state = json.loads((session_dir / "session_state.json").read_text())
    run_num = state.get("current_run", 1)
    new_run = run_num + 1

    # Advance state to ITERATE
    session_script = REPO_ROOT / ".claude" / "rl-training" / "scripts" / "session.py"
    subprocess.run(
        ["uv", "run", str(session_script), "update", str(session_dir),
         "--set", "phase=ITERATE", "--set", f"current_run={new_run}"],
        capture_output=True, text=True, cwd=str(REPO_ROOT), timeout=15,
    )
    subprocess.run(
        ["uv", "run", str(session_script), "add-iteration", str(session_dir),
         "--run", str(run_num), "--result", f"Interrupted by user: {mod.get('summary', '')}"],
        capture_output=True, text=True, cwd=str(REPO_ROOT), timeout=15,
    )

    # Create next run dir
    next_run_dir = session_dir / f"run_{new_run:03d}"
    next_run_dir.mkdir(exist_ok=True)

    kill_out = (kill_result.stdout + kill_result.stderr).strip()[:200]
    return (
        f"Training killed. phase → ITERATE (run {run_num} → {new_run})\n"
        f"Modification: {mod.get('summary', '(unknown)')}\n"
        f"Kill output: {kill_out or 'ok'}"
    )


def spawn_iterate_agent(branch_sanitized: str) -> bool:
    """Spawn the iterate agent as a background claude process after !apply.

    Returns True if the prompt file existed and the process was started.
    The keepalive cron (every 13 min) recreates the monitoring cron once
    phase flips back to MONITOR — no cron management needed here.
    """
    prompt_file = REPO_ROOT / ".claude" / "rl-training" / "sessions" / f"iterate_agent_{branch_sanitized}.md"
    if not prompt_file.exists():
        return False
    log_file = SESSION_DIR / branch_sanitized / "iterate_agent.log"
    log_file.parent.mkdir(parents=True, exist_ok=True)
    subprocess.Popen(
        [CLAUDE_BIN, "--print", "--dangerously-skip-permissions",
         "--model", ITERATE_CLAUDE_MODEL, prompt_file.read_text()],
        stdout=open(log_file, "w"),
        stderr=subprocess.STDOUT,
        cwd=str(REPO_ROOT),
    )
    return True


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
                "`!ask <question>` — ask anything about the current training run\n"
                "`!pending [branch]` — show queued modification requests\n"
                "`!apply [branch]` — interrupt training now to apply queued modification\n"
                "`!help` — this message\n\n"
                "**Tip:** questions (ending in `?` or starting with what/how/why/…) "
                "are answered by the bot. Messages starting with increase/decrease/set/change/… "
                "are queued as modification requests. Other free-text is stored as human feedback."
            )
            await message.channel.send(help_text)

        elif cmd == "!ask":
            if len(parts) < 2:
                await message.channel.send("Usage: `!ask <your question>`")
                return
            question = content[len("!ask"):].strip()
            sessions = get_sessions()
            active = [s for s in sessions if s.get("phase") in ("MONITOR", "ITERATE", "LAUNCH")]
            if not active:
                await message.channel.send("No active training session to ask about.")
                return
            session = active[-1]
            branch_sanitized = session.get("branch", "?").replace("/", "--")
            await message.add_reaction("⏳")
            ctx = await asyncio.get_event_loop().run_in_executor(
                None, _load_training_context, branch_sanitized
            )
            answer = await asyncio.get_event_loop().run_in_executor(
                None, answer_question, question, ctx
            )
            await message.remove_reaction("⏳", client.user)
            await message.channel.send(answer)

        elif cmd == "!pending":
            sessions = get_sessions()
            active = [s for s in sessions if s.get("phase") in ("MONITOR", "ITERATE", "LAUNCH")]
            branch_sanitized = parts[1].replace("/", "--") if len(parts) > 1 else (active[-1].get("branch", "?").replace("/", "--") if active else None)
            if not branch_sanitized:
                await message.channel.send("No active session found.")
                return
            queue_file = SESSION_DIR / branch_sanitized / "pending_modification.json"
            if not queue_file.exists():
                await message.channel.send(f"No pending modifications for `{branch_sanitized}`.")
                return
            queue = json.loads(queue_file.read_text())
            if not queue:
                await message.channel.send("Pending modification file is empty.")
                return
            lines = [f"**Pending modifications for `{branch_sanitized}`** ({len(queue)} queued)"]
            for i, entry in enumerate(queue, 1):
                urgency_icon = "⚡" if entry.get("urgency") == "immediate" else "⏳"
                params = ", ".join(entry.get("parameters", [])) or "unspecified"
                lines.append(f"{i}. {urgency_icon} **{entry.get('summary', '?')}**")
                lines.append(f"   Parameters: {params}")
                lines.append(f"   Note: {entry.get('notes', '')[:120]}")
            await message.channel.send("\n".join(lines))

        elif cmd == "!apply":
            # Interrupt training now to apply queued modification
            sessions = get_sessions()
            active = [s for s in sessions if s.get("phase") == "MONITOR"]
            if not active:
                await message.channel.send("No session in MONITOR phase to interrupt.")
                return
            branch_sanitized = parts[1].replace("/", "--") if len(parts) > 1 else active[-1].get("branch", "?").replace("/", "--")
            await message.channel.send(f"Interrupting `{branch_sanitized}` to apply pending modification…")
            result = await asyncio.get_event_loop().run_in_executor(
                None, apply_modification_now, branch_sanitized
            )
            await message.channel.send(f"```\n{result}\n```")
            if result.startswith("Training killed."):
                spawned = spawn_iterate_agent(branch_sanitized)
                if spawned:
                    await message.channel.send("ITERATE agent started — implementing changes in background with Sonnet. Monitoring will resume automatically once training relaunches.")
                else:
                    await message.channel.send("Phase is now **ITERATE** — keepalive agent will auto-implement changes and relaunch training within 13 minutes.")

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
    # Free-text → question (Haiku Q&A) or human feedback                  #
    # ------------------------------------------------------------------ #
    sessions = get_sessions()
    active = [s for s in sessions if s.get("phase") in ("MONITOR", "ITERATE", "LAUNCH")]
    if not active:
        return  # nothing to work with

    if len(active) > 1:
        session = active[-1]
    else:
        session = active[0]

    branch = session.get("branch", "?")
    branch_sanitized = branch.replace("/", "--")
    run_num = session.get("current_run", 1)

    await message.add_reaction("⏳")

    # Route: question → Haiku Q&A, modification request → queue, other → feedback
    if is_question(content):
        ctx = await asyncio.get_event_loop().run_in_executor(
            None, _load_training_context, branch_sanitized
        )
        answer = await asyncio.get_event_loop().run_in_executor(
            None, answer_question, content, ctx
        )
        await message.remove_reaction("⏳", client.user)
        await message.channel.send(answer)
        return

    if is_modification_request(content):
        ctx = await asyncio.get_event_loop().run_in_executor(
            None, _load_training_context, branch_sanitized
        )
        parsed = await asyncio.get_event_loop().run_in_executor(
            None, parse_modification_request, content, ctx
        )
        await asyncio.get_event_loop().run_in_executor(
            None, queue_modification, branch_sanitized, run_num, parsed
        )
        await message.remove_reaction("⏳", client.user)
        urgency = parsed.get("urgency", "next_iterate")
        params = parsed.get("parameters", [])
        param_str = ", ".join(f"`{p}`" for p in params) if params else "_(unspecified)_"
        urgency_str = "⚡ **immediate** (will trigger ITERATE now)" if urgency == "immediate" else "⏳ **queued** (applied at next ITERATE cycle)"
        err = parsed.get("error")
        reply = (
            f"Modification request queued for `{branch}`\n"
            f"Summary: {parsed.get('summary', content[:80])}\n"
            f"Parameters: {param_str}\n"
            f"Urgency: {urgency_str}"
        )
        if err:
            reply += f"\n⚠️ Parse warning: {err}"
        await message.channel.send(reply)
        return

    # Feedback path
    if len(active) > 1:
        branches = ", ".join(f"`{s.get('branch', '?')}`" for s in active)
        await message.channel.send(
            f"Multiple active sessions ({branches}). "
            "Attaching feedback to the most recent one."
        )

    tags_map = _load_feedback_tags()
    classified = await asyncio.get_event_loop().run_in_executor(
        None, classify_feedback, content, tags_map
    )

    tags = classified.get("tags", [])
    notes = classified.get("notes", content)
    err = classified.get("error")

    if err:
        await message.channel.send(f"⚠️ Classification failed ({err}) — storing as raw note.")

    await asyncio.get_event_loop().run_in_executor(
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

LOCKFILE = REPO_ROOT / ".discord_bot.pid"


def acquire_lock() -> None:
    if LOCKFILE.exists():
        old_pid = LOCKFILE.read_text().strip()
        try:
            os.kill(int(old_pid), 0)
            raise SystemExit(f"Another instance is already running (PID {old_pid}). Kill it first or delete {LOCKFILE}.")
        except (ProcessLookupError, ValueError):
            pass  # stale lockfile — safe to overwrite
    LOCKFILE.write_text(str(os.getpid()))


def release_lock() -> None:
    try:
        LOCKFILE.unlink()
    except FileNotFoundError:
        pass


def main():
    acquire_lock()
    try:
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
    finally:
        release_lock()


if __name__ == "__main__":
    main()
