#!/usr/bin/env python3
"""Deterministic KEEP/BAD/FINISH decision engine.

Usage:
    uv run .claude/rl-training/scripts/decide.py <run-dir> --monitor <M> --config <config.md> [--task-config <monitor_config.md>]

Output (stdout): JSON with decision, reasons, and updated counters.
    {"decision": "KEEP|BAD|FINISH", "reasons": [...], "consecutive_bad": N, "eval_requested": false, "notification": "..."}
"""

import argparse
import json
import re
import sys
from pathlib import Path


def load_json(path):
    p = Path(path)
    if not p.exists():
        return None
    return json.loads(p.read_text())


def parse_thresholds(config_text):
    """Extract decision criteria from config.md."""
    thresholds = {}
    for key in ("Kill threshold", "Max iterations"):
        m = re.search(rf"^- {key}:\s*(\d+)", config_text, re.MULTILINE)
        if m:
            thresholds[key.lower().replace(" ", "_")] = int(m.group(1))
    return thresholds


def parse_task_config(path):
    """Extract quality thresholds from monitor_config.md."""
    if path is None:
        return None
    p = Path(path)
    if not p.exists():
        return None
    text = p.read_text()
    config = {}
    for key in ("quality_score_bad_threshold", "quality_declining_monitors", "quality_finish_minimum"):
        m = re.search(rf"^- {key}:\s*(.+)$", text, re.MULTILINE)
        if m:
            val = m.group(1).strip()
            config[key] = float(val) if "." in val else int(val)
    m = re.search(r"^- reward_vs_quality_divergence:\s*(.+)$", text, re.MULTILINE)
    if m:
        config["reward_vs_quality_divergence"] = m.group(1).strip().lower() == "true"
    return config


def compute_decision(run_dir, monitor_num, config_path, task_config_path, session_dir):
    run_dir = Path(run_dir)
    config_text = Path(config_path).read_text()
    thresholds = parse_thresholds(config_text)
    task_config = parse_task_config(task_config_path)

    kill_threshold = thresholds.get("kill_threshold", 2)

    # Load session state
    state = load_json(Path(session_dir) / "session_state.json") if session_dir else {}
    consecutive_bad = state.get("consecutive_bad", 0) if state else 0

    # Load current and previous derived metrics
    pad = lambda n: f"{n:03d}"
    current_derived = load_json(run_dir / f"derived_metrics_{pad(monitor_num)}.json")
    prev_derived = load_json(run_dir / f"derived_metrics_{pad(monitor_num - 1)}.json") if monitor_num > 1 else None

    # Load current and previous raw metrics from monitor files
    current_raw = load_raw_from_monitor(run_dir / f"monitor_{pad(monitor_num)}.md")
    prev_raw = load_raw_from_monitor(run_dir / f"monitor_{pad(monitor_num - 1)}.md") if monitor_num > 1 else None

    reasons = []
    decision = "KEEP"
    eval_requested = False

    # Check if training step is advancing
    curr_step = current_raw.get("_step") if current_raw else None
    prev_step = prev_raw.get("_step") if prev_raw else None
    if curr_step is not None and prev_step is not None and curr_step <= prev_step:
        reasons.append(f"Training stalled at step {curr_step}")
        decision = "BAD"

    # Check main reward metric trends
    if current_raw and prev_raw:
        reward_keys = [k for k in current_raw if "Episode_Reward" in k and "total" in k.lower()]
        if not reward_keys:
            reward_keys = [k for k in current_raw if "Episode_Reward" in k]
        for rk in reward_keys[:1]:  # Use first matching reward key
            curr_val = current_raw.get(rk)
            prev_val = prev_raw.get(rk)
            if curr_val is not None and prev_val is not None and prev_val != 0:
                pct_change = (curr_val - prev_val) / abs(prev_val)
                if pct_change < -0.10:
                    reasons.append(f"Reward degraded {pct_change:.0%} ({rk})")
                    decision = "BAD"

    # Check quality metrics (if task config exists)
    if task_config and current_derived:
        quality = current_derived.get("quality_score")
        bad_threshold = task_config.get("quality_score_bad_threshold", 0.4)

        if quality is not None and quality < bad_threshold:
            reasons.append(f"Quality score {quality:.2f} below threshold {bad_threshold}")
            decision = "BAD"
            eval_requested = True

        # Check reward/quality divergence over N consecutive monitors
        declining_limit = task_config.get("quality_declining_monitors", 2)
        if task_config.get("reward_vs_quality_divergence") and monitor_num >= declining_limit:
            declining_count = 0
            for m in range(monitor_num - declining_limit + 1, monitor_num + 1):
                d_curr = load_json(run_dir / f"derived_metrics_{pad(m)}.json")
                d_prev = load_json(run_dir / f"derived_metrics_{pad(m - 1)}.json") if m > 1 else None
                r_curr = load_raw_from_monitor(run_dir / f"monitor_{pad(m)}.md")
                r_prev = load_raw_from_monitor(run_dir / f"monitor_{pad(m - 1)}.md") if m > 1 else None
                if not d_curr or not d_prev or not r_curr or not r_prev:
                    break
                q_curr, q_prev = d_curr.get("quality_score"), d_prev.get("quality_score")
                if q_curr is None or q_prev is None:
                    break
                q_drop = (q_prev - q_curr) / max(q_prev, 1e-6) > 0.05
                r_up = False
                for rk in [k for k in r_curr if "Episode_Reward" in k][:1]:
                    cv, pv = r_curr.get(rk), r_prev.get(rk)
                    if cv is not None and pv is not None and pv != 0:
                        r_up = (cv - pv) / abs(pv) > 0.05
                if q_drop and r_up:
                    declining_count += 1
            if declining_count >= declining_limit:
                reasons.append(f"Reward improving but quality declining for {declining_count} consecutive monitors")
                decision = "BAD"

    # Check for FINISH (plateau + quality ok)
    if decision == "KEEP" and monitor_num >= 3:
        recent_rewards = []
        for m in range(max(1, monitor_num - 2), monitor_num + 1):
            raw = load_raw_from_monitor(run_dir / f"monitor_{pad(m)}.md")
            if raw:
                for rk in [k for k in raw if "Episode_Reward" in k][:1]:
                    if raw.get(rk) is not None:
                        recent_rewards.append(raw[rk])
        if len(recent_rewards) >= 3:
            max_r, min_r = max(recent_rewards), min(recent_rewards)
            avg = sum(recent_rewards) / len(recent_rewards)
            if avg != 0 and (max_r - min_r) / abs(avg) < 0.02:
                quality_ok = True
                if task_config and current_derived:
                    q = current_derived.get("quality_score")
                    if q is not None and q < task_config.get("quality_finish_minimum", 0.7):
                        quality_ok = False
                if quality_ok:
                    reasons.append("Reward plateaued (< 2% variation over 3 monitors)")
                    decision = "FINISH"

    # If still KEEP, give reason
    if decision == "KEEP" and not reasons:
        reasons.append("Metrics stable or improving")

    # Update consecutive_bad
    if decision == "BAD":
        consecutive_bad += 1
    else:
        consecutive_bad = 0

    # Check if BAD should escalate to KILL
    should_kill = decision == "BAD" and consecutive_bad >= kill_threshold

    # Format notification message
    notification = format_notification(decision, reasons, monitor_num, current_raw, current_derived, should_kill)

    return {
        "decision": decision,
        "should_kill": should_kill,
        "reasons": reasons,
        "consecutive_bad": consecutive_bad,
        "eval_requested": eval_requested,
        "notification": notification,
    }


def load_raw_from_monitor(path):
    """Extract RAW_METRICS JSON from a monitor markdown file."""
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


def format_notification(decision, reasons, monitor_num, raw, derived, should_kill):
    """Format a concise notification message."""
    parts = [f"Monitor {monitor_num}"]
    if raw and "_step" in raw:
        parts[0] += f" (step {int(raw['_step'])})"
    parts.append(f"Decision: {'KILL' if should_kill else decision}")
    if derived and derived.get("quality_score") is not None:
        parts.append(f"Quality: {derived['quality_score']:.2f}")
        coverage = derived.get("coverage")
        if coverage is not None and coverage < 1.0:
            parts[-1] += f" ({coverage:.0%} coverage)"
    for r in reasons:
        parts.append(f"  - {r}")
    return "\n".join(parts)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", help="Path to run directory (e.g., logs/sessions/branch/run_001)")
    parser.add_argument("--monitor", type=int, required=True, help="Current monitor number")
    parser.add_argument("--config", required=True, help="Path to config.md")
    parser.add_argument("--task-config", default=None, help="Path to task monitor_config.md")
    parser.add_argument("--session-dir", default=None, help="Path to session directory")
    args = parser.parse_args()

    result = compute_decision(args.run_dir, args.monitor, args.config, args.task_config, args.session_dir)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
