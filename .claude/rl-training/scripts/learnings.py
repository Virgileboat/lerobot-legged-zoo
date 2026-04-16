#!/usr/bin/env python3
"""Gather structured run data for the learning agent.

Usage:
    uv run .claude/rl-training/scripts/learnings.py <session-dir> --run <N>

Reads run data (result.md, analysis.md, metrics, session_state.json, git diff)
and outputs a structured markdown report to stdout for the learning agent.

Note: Must be run from the project git root (git commands use no cwd override).
"""

import argparse
import glob
import json
import subprocess
import sys
from pathlib import Path


def load_text(path):
    p = Path(path)
    return p.read_text() if p.exists() else None


def load_json(path):
    p = Path(path)
    return json.loads(p.read_text()) if p.exists() else None


def load_raw_from_monitor(path):
    p = Path(path)
    if not p.exists():
        return None
    text = p.read_text()
    marker = "<!-- RAW_METRICS:"
    if marker not in text:
        return None
    start = text.index(marker) + len(marker)
    end = text.index("-->", start)
    return json.loads(text[start:end])


def collect_metrics_trajectory(run_dir):
    """Collect quality scores across all derived_metrics files."""
    files = sorted(glob.glob(str(run_dir / "derived_metrics_*.json")))
    trajectory = []
    for f in files:
        data = load_json(f)
        if data and data.get("quality_score") is not None:
            entry = {"quality_score": data["quality_score"]}
            for k, v in data.items():
                if k not in ("quality_score", "coverage", "missing") and isinstance(v, (int, float)):
                    entry[k] = v
            trajectory.append(entry)
    return trajectory


def get_reward_trend(run_dir):
    """Determine reward trend from raw metrics embedded in monitor_NNN.md files."""
    files = sorted(glob.glob(str(run_dir / "monitor_*.md")))
    rewards = []
    for f in files:
        data = load_raw_from_monitor(f)
        if not data:
            continue
        reward_keys = [k for k in data if "Episode_Reward" in k and isinstance(data[k], (int, float))]
        if reward_keys:
            total = sum(data[k] for k in reward_keys)
            rewards.append(total)
    if len(rewards) < 2:
        return "insufficient data"
    change = (rewards[-1] - rewards[0]) / (abs(rewards[0]) + 1e-8)
    if change > 0.05:
        return "improving"
    elif change < -0.05:
        return "degrading"
    return "plateauing"


def get_git_diff_summary(branch):
    """Get a summary of code changes on the branch."""
    try:
        result = subprocess.run(
            ["git", "log", "--oneline", "--no-merges", "-10", branch, "--not", "main"],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode != 0 or not result.stdout.strip():
            return None
        commits = result.stdout.strip()
        diff = subprocess.run(
            ["git", "diff", "--stat", "main..." + branch],
            capture_output=True, text=True, timeout=10
        )
        return f"Commits:\n{commits}\n\nFiles changed:\n{diff.stdout.strip()}" if diff.returncode == 0 else commits
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None


def find_similar_iterations(iterations, current_run, current_tags):
    """Find past iterations with similar feedback tags or diagnosis patterns."""
    similar = []
    for it in iterations:
        if it["run"] >= current_run:
            continue
        fb = it.get("human_feedback")
        if fb and fb.get("tags"):
            overlap = set(fb["tags"]) & set(current_tags)
            if overlap:
                similar.append(f"Run {it['run']}: tags {list(overlap)} — {it.get('result', 'no result')}")
    return similar


def generate_report(session_dir, run_num):
    session_dir = Path(session_dir)
    run_dir = session_dir / f"run_{run_num:03d}"
    state = load_json(session_dir / "session_state.json")

    if not state:
        print("ERROR: session_state.json not found", file=sys.stderr)
        sys.exit(1)

    lines = []

    # Run Summary
    lines.append("## Run Summary")
    lines.append(f"- Goal: {state.get('goal', 'unknown')}")
    lines.append(f"- Branch: {state.get('branch', 'unknown')}")
    lines.append(f"- Run: {run_num} of {state.get('current_run', '?')} total iterations")
    lines.append(f"- Phase: {state.get('phase', 'unknown')}")

    result_text = load_text(run_dir / "result.md")
    if result_text:
        for line in result_text.splitlines():
            if line.startswith("**Reason:**"):
                lines.append(f"- Reason: {line.replace('**Reason:**', '').strip()}")
                break
            if line.startswith("**Killed at:**"):
                lines.append(f"- {line.replace('**', '').strip()}")
    lines.append("")

    # Metrics Trajectory
    trajectory = collect_metrics_trajectory(run_dir)
    lines.append("## Metrics Trajectory")
    if trajectory:
        first = trajectory[0]
        last = trajectory[-1]
        lines.append(f"- Starting quality_score: {first['quality_score']:.3f} -> Final: {last['quality_score']:.3f}")
        sub_keys = [k for k in last if k != "quality_score"]
        for k in sub_keys:
            start_val = first.get(k)
            end_val = last.get(k)
            if start_val is not None and end_val is not None:
                lines.append(f"- {k}: {start_val:.3f} -> {end_val:.3f}")
    lines.append(f"- Reward trend: {get_reward_trend(run_dir)}")
    lines.append("")

    # Diagnosis & Fix
    lines.append("## Diagnosis & Fix")
    analysis = load_text(run_dir / "analysis.md")
    if analysis:
        lines.append(f"- Hypothesis: (from analysis.md)")
        for aline in analysis.splitlines():
            if "hypothesis" in aline.lower() or "root cause" in aline.lower() or "because" in aline.lower():
                lines.append(f"  {aline.strip()}")
    else:
        lines.append("- No analysis.md found (training may have finished successfully)")

    diff_summary = get_git_diff_summary(state.get("branch", ""))
    if diff_summary:
        lines.append(f"- Code changes:\n{diff_summary}")
    else:
        lines.append("- Code changes: not available")

    # Did the fix help?
    if trajectory and len(trajectory) >= 2:
        delta = trajectory[-1]["quality_score"] - trajectory[0]["quality_score"]
        if delta > 0.05:
            lines.append("- Fix assessment: quality improved")
        elif delta < -0.05:
            lines.append("- Fix assessment: quality degraded")
        else:
            lines.append("- Fix assessment: inconclusive (quality unchanged)")
    else:
        lines.append("- Fix assessment: insufficient metrics data")
    lines.append("")

    # Human Feedback
    lines.append("## Human Feedback")
    current_tags = []
    iterations = state.get("iterations", [])
    for it in iterations:
        if it["run"] == run_num:
            fb = it.get("human_feedback")
            if fb and (fb.get("tags") or fb.get("notes")):
                current_tags = fb.get("tags", [])
                lines.append(f"- Tags: {current_tags}")
                lines.append(f"- Notes: {fb.get('notes', '')}")
            else:
                lines.append("- No human feedback for this run")
            break
    else:
        lines.append("- No iteration entry found for this run")
    lines.append("")

    # Iteration History Context
    lines.append("## Iteration History Context")
    if iterations:
        lines.append(f"- Total iterations so far: {len(iterations)}")
        for it in iterations:
            fb = it.get("human_feedback")
            fb_str = f" [tags: {fb['tags']}]" if fb and fb.get("tags") else ""
            lines.append(f"  - Run {it['run']}: {it.get('result', 'no result')}{fb_str}")
        similar = find_similar_iterations(iterations, run_num, current_tags)
        if similar:
            lines.append("- Similar past iterations:")
            for s in similar:
                lines.append(f"  - {s}")
    else:
        lines.append("- No previous iterations")
    lines.append("")

    print("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description="Gather run data for the learning agent")
    parser.add_argument("session_dir", help="Path to session directory")
    parser.add_argument("--run", type=int, required=True, help="Run number to analyze")
    args = parser.parse_args()

    generate_report(args.session_dir, args.run)


if __name__ == "__main__":
    main()
