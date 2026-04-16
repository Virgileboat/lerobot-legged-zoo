#!/usr/bin/env python3
"""Compute task-specific derived quality metrics from raw WandB data.

Usage:
    uv run .claude/rl-training/tasks/locomotion/monitor_metrics.py <raw_metrics.json> \
        [--previous <prev_derived.json>] [--config <monitor_config.md>]

Input:  JSON file of raw WandB metrics (from <!-- RAW_METRICS:... --> in monitor output)
Output: JSON to stdout with derived metrics + markdown summary to stderr
"""

import argparse
import json
import re
import sys
from pathlib import Path


def parse_monitor_config(config_path):
    text = Path(config_path).read_text()

    left_match  = re.search(r"^- left:\s*\[(.+)\]",  text, re.MULTILINE)
    right_match = re.search(r"^- right:\s*\[(.+)\]", text, re.MULTILINE)
    left_joints  = [j.strip() for j in left_match.group(1).split(",")]  if left_match  else []
    right_joints = [j.strip() for j in right_match.group(1).split(",")] if right_match else []

    loco_match = re.search(r"^- locomotion:\s*\[(.+)\]",     text, re.MULTILINE)
    reg_match  = re.search(r"^- regularization:\s*\[(.+)\]", text, re.MULTILINE)
    loco_prefixes = [p.strip() for p in loco_match.group(1).split(",")] if loco_match else []
    reg_prefixes  = [p.strip() for p in reg_match.group(1).split(",")]  if reg_match  else []

    weights = {}
    for line in text.splitlines():
        m = re.match(r"\|\s*(\w+)\s*\|.*\|\s*([\d.]+)\s*\|", line)
        if m and m.group(1) not in ("Metric",):
            weights[m.group(1)] = float(m.group(2))

    thresholds = {}
    for key in ("quality_score_bad_threshold", "quality_declining_monitors", "quality_finish_minimum"):
        m = re.search(rf"^- {key}:\s*(.+)$", text, re.MULTILINE)
        if m:
            val = m.group(1).strip()
            thresholds[key] = float(val) if "." in val else int(val)

    return {
        "left_joints":    left_joints,
        "right_joints":   right_joints,
        "loco_prefixes":  loco_prefixes,
        "reg_prefixes":   reg_prefixes,
        "weights":        weights,
        "thresholds":     thresholds,
    }


def compute_symmetry_ratio(metrics, left_joints, right_joints):
    """Compare left vs right joint torques. Returns 0-1 (1 = symmetric)."""
    if not left_joints or not right_joints:
        return None
    left_vals, right_vals = [], []
    for lj, rj in zip(left_joints, right_joints):
        lk = f"Torque/{lj}_mean"
        rk = f"Torque/{rj}_mean"
        if lk in metrics and rk in metrics:
            left_vals.append(abs(metrics[lk]))
            right_vals.append(abs(metrics[rk]))
    if not left_vals:
        return None
    total = sum(left_vals) + sum(right_vals)
    if total < 1e-6:
        return 1.0
    diff = sum(abs(l - r) for l, r in zip(left_vals, right_vals))
    return max(0.0, 1.0 - diff / total)


def compute_action_smoothness(metrics):
    """Use action_fft_band_le_3hz_ratio reward as the smoothness score directly.

    Falls back to action_rate_l2 if FFT metric is not available.
    """
    # Prefer the FFT band ratio reward (already 0-1)
    for k in metrics:
        if "action_fft" in k.lower() and "Episode_Reward" in k:
            val = metrics[k]
            if val is not None:
                return max(0.0, min(1.0, float(val)))
    # Fallback: action_rate_l2 (negative penalty, range ~[-1, 0])
    for k in metrics:
        if "action_rate" in k.lower() and "Episode_Reward" in k and "hipz" not in k:
            val = metrics[k]
            if val is not None:
                return max(0.0, min(1.0, 1.0 + float(val)))
    return None


def compute_survival_ratio(metrics):
    """Fraction of episodes ending in timeout (survived)."""
    timeout = None
    total = 0.0
    for k, v in metrics.items():
        if not k.startswith("Episode_Termination/") or v is None:
            continue
        if "time_out" in k:
            timeout = v
        total += v
    if timeout is None or total < 1e-6:
        return None
    return float(timeout / total)


def compute_reward_balance(metrics, loco_prefixes, reg_prefixes):
    """Ratio of locomotion vs regularization reward magnitudes (0.5 = balanced)."""
    loco_sum, reg_sum = 0.0, 0.0
    loco_count = reg_count = 0
    for k, v in metrics.items():
        if not k.startswith("Episode_Reward/") or v is None:
            continue
        name = k.replace("Episode_Reward/", "")
        if any(name.startswith(p) or name == p for p in loco_prefixes):
            loco_sum += abs(v); loco_count += 1
        elif any(name.startswith(p) or name == p for p in reg_prefixes):
            reg_sum += abs(v); reg_count += 1
    if loco_count == 0 or reg_count == 0:
        return None
    total = loco_sum + reg_sum
    if total < 1e-6:
        return 0.5
    ratio = loco_sum / total
    return max(0.0, min(1.0, 1.0 - 2.0 * abs(ratio - 0.5)))


def compute_quality_score(sub_scores, weights):
    total_w = weighted_sum = 0.0
    missing = []
    for name in weights:
        score = sub_scores.get(name)
        if score is not None:
            weighted_sum += score * weights[name]
            total_w += weights[name]
        else:
            missing.append(name)
    expected_w = sum(weights.values())
    coverage = total_w / expected_w if expected_w > 1e-6 else 0.0
    quality = max(0.0, min(1.0, weighted_sum / total_w)) if total_w > 1e-6 else None
    return quality, coverage, missing


def format_markdown(derived, previous):
    lines = ["\n## Quality Metrics (Tier 1)"]
    for name, val in derived.items():
        if name in ("coverage", "missing"):
            continue
        if val is None:
            lines.append(f"- **{name}**: N/A")
            continue
        if not isinstance(val, (int, float)):
            continue
        trend = ""
        if previous and name in previous and isinstance(previous.get(name), (int, float)):
            diff = val - previous[name]
            if abs(diff) > 0.01:
                trend = f" ({'↑' if diff > 0 else '↓'} {abs(diff):.2f})"
            else:
                trend = " (stable)"
        lines.append(f"- **{name}**: {val:.3f}{trend}")
    cov = derived.get("coverage", 1.0)
    missing = derived.get("missing", [])
    if cov < 1.0:
        lines.append(f"- **coverage**: {cov:.0%} — missing: {', '.join(missing)}")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("raw_metrics", help="Path to raw metrics JSON")
    parser.add_argument("--previous", help="Path to previous derived metrics JSON")
    parser.add_argument("--config", default=".claude/rl-training/tasks/locomotion/monitor_config.md")
    args = parser.parse_args()

    metrics = json.loads(Path(args.raw_metrics).read_text())
    config = parse_monitor_config(args.config)

    previous = None
    if args.previous:
        p = Path(args.previous)
        if p.exists():
            previous = json.loads(p.read_text())

    sub_scores = {
        "symmetry_ratio":   compute_symmetry_ratio(metrics, config["left_joints"], config["right_joints"]),
        "action_smoothness": compute_action_smoothness(metrics),
        "survival_ratio":   compute_survival_ratio(metrics),
        "reward_balance":   compute_reward_balance(metrics, config["loco_prefixes"], config["reg_prefixes"]),
    }

    quality, coverage, missing = compute_quality_score(sub_scores, config["weights"])
    derived = {**sub_scores, "quality_score": quality, "coverage": coverage, "missing": missing}

    print(json.dumps(derived))
    print(format_markdown(derived, previous), file=sys.stderr)


if __name__ == "__main__":
    main()
