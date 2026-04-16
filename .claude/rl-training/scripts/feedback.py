#!/usr/bin/env python3
"""Persist structured human feedback to result.md and session_state.json.

Usage:
    uv run .claude/rl-training/scripts/feedback.py <session-dir> --run <N> --tags <t1,t2> [--notes "<text>"]
    uv run .claude/rl-training/scripts/feedback.py <session-dir> --run <N> --list-tags --task-config <monitor_config.md>

Updates:
  - run_NNN/result.md → Human Assessment section
  - session_state.json → iterations[N].human_feedback
"""

import argparse
import json
import re
import sys
from pathlib import Path


def parse_valid_tags(config_path):
    """Extract valid feedback tags from monitor_config.md."""
    text = Path(config_path).read_text()
    tags = {}
    in_tags = False
    for line in text.splitlines():
        if "## Human Feedback Tags" in line:
            in_tags = True
            continue
        if in_tags and line.startswith("##"):
            break
        if in_tags:
            m = re.match(r"^- (\w+):\s*(.+)$", line)
            if m:
                tags[m.group(1)] = m.group(2).strip()
    return tags


def parse_tag_metric_mapping(config_path):
    """Extract tag-to-metric mapping from monitor_config.md."""
    text = Path(config_path).read_text()
    mapping = {}
    in_mapping = False
    for line in text.splitlines():
        if "## Tag-to-Metric Mapping" in line:
            in_mapping = True
            continue
        if in_mapping and line.startswith("##"):
            break
        if in_mapping:
            m = re.match(r"^- (\w+)\s*→\s*(.+)$", line)
            if m:
                target = m.group(2).strip()
                mapping[m.group(1)] = None if "no metric" in target else target
    return mapping


def update_result_md(run_dir, run_num, tags, notes):
    """Update the Human Assessment section in result.md."""
    result_path = run_dir / f"run_{run_num:03d}" / "result.md"
    if not result_path.exists():
        print(f"WARNING: {result_path} not found — skipping result.md update", file=sys.stderr)
        return
    text = result_path.read_text()
    tag_str = json.dumps(tags)
    new_section = f"## Human Assessment\nTags: {tag_str}\nNotes: {notes or 'No feedback yet.'}"
    if "## Human Assessment" in text:
        text = re.sub(
            r"## Human Assessment\n.*?(?=\n## |\Z)",
            new_section + "\n",
            text,
            flags=re.DOTALL,
        )
    else:
        text = text.rstrip() + "\n\n" + new_section + "\n"
    result_path.write_text(text)
    print(f"Updated {result_path}")


def update_session_state(session_dir, run_num, tags, notes):
    """Update session_state.json iterations[N].human_feedback."""
    state_path = Path(session_dir) / "session_state.json"
    if not state_path.exists():
        print(f"ERROR: {state_path} not found", file=sys.stderr)
        sys.exit(1)
    state = json.loads(state_path.read_text())
    feedback = {"tags": tags, "notes": notes or ""}
    for it in state.get("iterations", []):
        if it["run"] == run_num:
            it["human_feedback"] = feedback
            state_path.write_text(json.dumps(state, indent=2) + "\n")
            print(f"Updated session_state.json iteration for run {run_num}")
            return
    print(f"WARNING: no iteration for run {run_num} in session_state.json — adding one", file=sys.stderr)
    state.setdefault("iterations", []).append({
        "run": run_num,
        "result": "(feedback added retroactively)",
        "human_feedback": feedback,
    })
    state_path.write_text(json.dumps(state, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("session_dir", help="Path to session directory")
    parser.add_argument("--run", type=int, required=True)
    parser.add_argument("--tags", default="", help="Comma-separated feedback tags")
    parser.add_argument("--notes", default="", help="Free-text notes")
    parser.add_argument("--task-config", default=None, help="Path to monitor_config.md (for validation)")
    parser.add_argument("--list-tags", action="store_true", help="List valid tags and exit")
    args = parser.parse_args()

    if args.list_tags:
        if not args.task_config:
            print("ERROR: --task-config required with --list-tags", file=sys.stderr)
            sys.exit(1)
        valid = parse_valid_tags(args.task_config)
        for tag, desc in valid.items():
            print(f"  {tag}: {desc}")
        return

    tags = [t.strip() for t in args.tags.split(",") if t.strip()]

    # Validate tags if config provided
    if args.task_config and tags:
        valid = parse_valid_tags(args.task_config)
        invalid = [t for t in tags if t not in valid]
        if invalid:
            print(f"WARNING: unknown tags (not in config): {', '.join(invalid)}", file=sys.stderr)

    update_result_md(Path(args.session_dir), args.run, tags, args.notes)
    update_session_state(args.session_dir, args.run, tags, args.notes)

    # Report tag-to-metric gaps if config has mapping
    if args.task_config:
        mapping = parse_tag_metric_mapping(args.task_config)
        unmapped = [t for t in tags if t in mapping and mapping[t] is None]
        if unmapped:
            print(f"NOTE: tags with no corresponding metric (Tier 3 candidates): {', '.join(unmapped)}", file=sys.stderr)


if __name__ == "__main__":
    main()
