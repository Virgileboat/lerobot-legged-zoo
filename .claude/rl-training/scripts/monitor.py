#!/usr/bin/env python3
"""Fetch wandb metrics and output a concise markdown summary.

Usage:
    uv run .claude/rl-training/scripts/monitor.py <run-path> [--previous <file>] [--categories <cat1,cat2,...>]
"""

import argparse
import json
import math
import sys
from pathlib import Path

import wandb


def trend(current, previous, key):
    if previous is None or key not in previous or previous[key] is None:
        return ""
    if current.get(key) is None:
        return ""
    diff = current[key] - previous[key]
    if abs(diff) < 1e-6:
        return " (stable)"
    return f" ({'↑' if diff > 0 else '↓'} {abs(diff):.4f})"


def format_summary(run, latest, previous_check, categories):
    status = run.state
    step = latest.get("_step", "?")

    lines = [
        f"# WandB Monitor — {run.name}",
        f"**Status**: {status} | **Step**: {step}",
        "",
        "## Metrics",
    ]

    for cat in categories:
        cat_keys = sorted(k for k in latest if k.startswith(cat) and latest[k] is not None)
        if not cat_keys:
            continue
        lines.append(f"\n### {cat.rstrip('/')}")
        for k in cat_keys:
            name = k.replace(cat, "")
            val = latest[k]
            t = trend(latest, previous_check, k)
            if isinstance(val, float):
                lines.append(f"- **{name}**: {val:.4f}{t}")
            else:
                lines.append(f"- **{name}**: {val}{t}")

    return "\n".join(lines)


def load_previous_metrics(path):
    p = Path(path)
    if not p.exists():
        return None
    text = p.read_text()
    marker = "<!-- RAW_METRICS:"
    if marker in text:
        start = text.index(marker) + len(marker)
        end = text.index("-->", start)
        return json.loads(text[start:end])
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_path", help="wandb run path: entity/project/runs/run_id")
    parser.add_argument("--previous", help="Path to previous monitor output for trend comparison")
    parser.add_argument("--categories", default="", help="Comma-separated metric category prefixes")
    parser.add_argument("--raw-output", help="Write raw metrics JSON to this file (in addition to embedding in markdown)")
    args = parser.parse_args()

    api = wandb.Api()
    try:
        run = api.run(args.run_path)
        run.update()
    except Exception as e:
        print(f"ERROR: Failed to fetch run {args.run_path}: {e}", file=sys.stderr)
        sys.exit(1)
    latest = dict(run.summary)

    if not latest or "_step" not in latest:
        print("No metrics available yet.")
        sys.exit(0)

    previous_check = None
    if args.previous:
        previous_check = load_previous_metrics(args.previous)

    categories = [c.strip() for c in args.categories.split(",") if c.strip()] if args.categories else []
    if not categories:
        prefixes = set()
        for k in latest:
            if "/" in k and not k.startswith("_"):
                prefixes.add(k.rsplit("/", 1)[0] + "/")
        categories = sorted(prefixes)

    summary = format_summary(run, latest, previous_check, categories)
    print(summary)

    raw = {k: float(v) for k, v in latest.items() if isinstance(v, (int, float)) and math.isfinite(float(v))}
    if args.raw_output:
        Path(args.raw_output).write_text(json.dumps(raw, indent=2) + "\n")
    print(f"\n<!-- RAW_METRICS:{json.dumps(raw)}-->")

    sys.exit(2 if run.state in ("killed", "crashed") else 0)


if __name__ == "__main__":
    main()
