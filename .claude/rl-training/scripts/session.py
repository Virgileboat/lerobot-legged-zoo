#!/usr/bin/env python3
"""Manage session_state.json reliably.

Usage:
    uv run .claude/rl-training/scripts/session.py get <session-dir> [--field <name>]
    uv run .claude/rl-training/scripts/session.py update <session-dir> --set key=value [--set key=value ...]
    uv run .claude/rl-training/scripts/session.py add-iteration <session-dir> --run <N> --result "<summary>"
    uv run .claude/rl-training/scripts/session.py set-feedback <session-dir> --run <N> --tags <t1,t2> [--notes "<text>"]
"""

import argparse
import json
import sys
from pathlib import Path


def load_state(session_dir):
    path = Path(session_dir) / "session_state.json"
    if not path.exists():
        print(f"ERROR: {path} not found", file=sys.stderr)
        sys.exit(1)
    return json.loads(path.read_text()), path


def save_state(state, path):
    path.write_text(json.dumps(state, indent=2) + "\n")


def cmd_get(args):
    state, _ = load_state(args.session_dir)
    if args.field:
        val = state.get(args.field)
        if val is None:
            print(f"ERROR: field '{args.field}' not found", file=sys.stderr)
            sys.exit(1)
        print(json.dumps(val) if isinstance(val, (dict, list)) else val)
    else:
        print(json.dumps(state, indent=2))


def cmd_update(args):
    state, path = load_state(args.session_dir)
    for pair in args.set:
        key, val = pair.split("=", 1)
        # Auto-type: int, float, bool, or string
        if val.lower() in ("true", "false"):
            state[key] = val.lower() == "true"
        else:
            try:
                state[key] = int(val)
            except ValueError:
                try:
                    state[key] = float(val)
                except ValueError:
                    state[key] = val
    save_state(state, path)
    print(json.dumps(state, indent=2))


def cmd_add_iteration(args):
    state, path = load_state(args.session_dir)
    state.setdefault("iterations", []).append({
        "run": args.run,
        "result": args.result,
        "human_feedback": None,
    })
    save_state(state, path)
    print(f"Added iteration for run {args.run}")


def cmd_set_feedback(args):
    state, path = load_state(args.session_dir)
    tags = [t.strip() for t in args.tags.split(",") if t.strip()] if args.tags else []
    feedback = {"tags": tags, "notes": args.notes or ""}
    for it in reversed(state.get("iterations", [])):
        if it["run"] == args.run:
            it["human_feedback"] = feedback
            save_state(state, path)
            print(f"Feedback set for run {args.run}: {json.dumps(feedback)}")
            return
    print(f"ERROR: no iteration found for run {args.run}", file=sys.stderr)
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command")

    p_get = sub.add_parser("get")
    p_get.add_argument("session_dir")
    p_get.add_argument("--field")

    p_update = sub.add_parser("update")
    p_update.add_argument("session_dir")
    p_update.add_argument("--set", action="append", required=True, help="key=value pair")

    p_iter = sub.add_parser("add-iteration")
    p_iter.add_argument("session_dir")
    p_iter.add_argument("--run", type=int, required=True)
    p_iter.add_argument("--result", required=True)

    p_fb = sub.add_parser("set-feedback")
    p_fb.add_argument("session_dir")
    p_fb.add_argument("--run", type=int, required=True)
    p_fb.add_argument("--tags", default="")
    p_fb.add_argument("--notes", default="")

    args = parser.parse_args()
    if not args.command:
        parser.print_help()
        sys.exit(1)

    {"get": cmd_get, "update": cmd_update, "add-iteration": cmd_add_iteration, "set-feedback": cmd_set_feedback}[args.command](args)


if __name__ == "__main__":
    main()
