#!/usr/bin/env python3
"""Generate result.md when a training run is killed.

Usage:
    uv run .claude/rl-training/scripts/generate_result.py <run-dir> --monitor-count <N> --reason "<why>" [--goal "<goal>"]

Reads: monitor_*.md, derived_metrics_*.json, eval_metrics.json
Writes: <run-dir>/result.md
"""

import argparse
import json
import sys
from pathlib import Path


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


def collect_quality_trend(run_dir, monitor_count):
    """Collect quality scores across all monitors."""
    trend = []
    for m in range(1, monitor_count + 1):
        derived = load_json(run_dir / f"derived_metrics_{m:03d}.json")
        if derived and derived.get("quality_score") is not None:
            trend.append({
                "monitor": m,
                "quality_score": derived["quality_score"],
                "coverage": derived.get("coverage", 1.0),
                "sub_scores": {k: v for k, v in derived.items()
                               if k not in ("quality_score", "coverage", "missing")
                               and isinstance(v, (int, float))},
            })
    return trend


def find_kill_triggers(run_dir, monitor_count):
    """Identify which quality sub-scores were below thresholds at kill time."""
    derived = load_json(run_dir / f"derived_metrics_{monitor_count:03d}.json")
    if not derived:
        return []
    triggers = []
    for k, v in derived.items():
        if k in ("quality_score", "coverage", "missing") or not isinstance(v, (int, float)):
            continue
        if v < 0.4:
            triggers.append(f"{k}: {v:.3f}")
    return triggers


def generate(run_dir, monitor_count, reason, goal):
    run_dir = Path(run_dir)
    lines = ["# Training Run Result", ""]

    if goal:
        lines.append(f"**Goal:** {goal}")

    # Get final step from last monitor
    raw = load_raw_from_monitor(run_dir / f"monitor_{monitor_count:03d}.md")
    step = int(raw["_step"]) if raw and "_step" in raw else "unknown"
    lines.append(f"**Killed at:** step {step}")
    lines.append(f"**Reason:** {reason}")
    lines.append(f"**Monitors:** {monitor_count}")
    lines.append("")

    # Key metrics at death
    if raw:
        lines.append("## Final Metrics")
        reward_keys = sorted(k for k in raw if "Episode_Reward" in k and isinstance(raw[k], (int, float)))
        for k in reward_keys[:10]:
            lines.append(f"- **{k}**: {raw[k]:.4f}")
        lines.append("")

    # Quality trend
    trend = collect_quality_trend(run_dir, monitor_count)
    if trend:
        lines.append("## Quality Score Trend")
        lines.append("| Monitor | Quality | Coverage | Key Sub-Scores |")
        lines.append("|---------|---------|----------|----------------|")
        for t in trend:
            subs = ", ".join(f"{k}: {v:.2f}" for k, v in list(t["sub_scores"].items())[:3])
            lines.append(f"| {t['monitor']} | {t['quality_score']:.3f} | {t['coverage']:.0%} | {subs} |")
        lines.append("")

    # Kill triggers
    triggers = find_kill_triggers(run_dir, monitor_count)
    if triggers:
        lines.append("## Kill Triggers (quality sub-scores below 0.4)")
        for t in triggers:
            lines.append(f"- {t}")
        lines.append("")

    # Eval results if available
    eval_data = load_json(run_dir / "eval_metrics.json")
    if eval_data:
        lines.append("## Evaluation Results")
        for scenario, data in eval_data.items():
            lines.append(f"### {scenario}")
            for k, v in data.items():
                if isinstance(v, float):
                    lines.append(f"- **{k}**: {v:.4f}")
                elif isinstance(v, list):
                    lines.append(f"- **{k}**: {v}")
            lines.append("")

    # Human Assessment (pre-filled)
    lines.append("## Human Assessment")
    lines.append("Tags: []")
    lines.append("Notes: No feedback yet.")
    lines.append("")

    result = "\n".join(lines)
    result_path = run_dir / "result.md"
    result_path.write_text(result)
    print(f"Generated {result_path}")
    return result_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", help="Path to run directory")
    parser.add_argument("--monitor-count", type=int, required=True)
    parser.add_argument("--reason", required=True, help="Why the run was killed")
    parser.add_argument("--goal", default="", help="Training goal")
    args = parser.parse_args()

    generate(args.run_dir, args.monitor_count, args.reason, args.goal)


if __name__ == "__main__":
    main()
