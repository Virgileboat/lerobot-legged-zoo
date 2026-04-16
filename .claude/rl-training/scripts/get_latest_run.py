#!/usr/bin/env python3
"""Find the latest wandb run for a project.

Usage:
    uv run .claude/rl-training/scripts/get_latest_run.py <wandb-project> [--state running] [--wait 300]

Output: run path (entity/project/runs/run_id) on stdout, exit 1 if not found.
"""

import argparse
import sys
import time

import wandb


def find_run(api, project, state, branch=None):
    filters = {"state": state}
    if branch:
        filters["config.branch"] = branch
    runs = api.runs(project, filters=filters, order="-created_at", per_page=1)
    for run in runs:
        return f"{project}/runs/{run.id}"
    if branch:
        print(f"WARNING: No {state} run found for branch '{branch}', trying without branch filter", file=sys.stderr)
        runs = api.runs(project, filters={"state": state}, order="-created_at", per_page=1)
        for run in runs:
            print(f"WARNING: Returning run from different branch: {run.name}", file=sys.stderr)
            return f"{project}/runs/{run.id}"
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("project", help="wandb project: entity/project")
    parser.add_argument("--state", default="running")
    parser.add_argument("--wait", type=int, default=0, help="Max seconds to poll")
    parser.add_argument("--branch", default=None, help="Filter by config.branch tag")
    args = parser.parse_args()

    try:
        api = wandb.Api()
    except Exception as e:
        print(f"ERROR: Failed to connect to wandb: {e}", file=sys.stderr)
        sys.exit(1)

    if args.wait <= 0:
        path = find_run(api, args.project, args.state, args.branch)
        if path:
            print(path)
        else:
            print("No run found", file=sys.stderr)
            sys.exit(1)
        return

    deadline = time.time() + args.wait
    while time.time() < deadline:
        path = find_run(api, args.project, args.state, args.branch)
        if path:
            print(path)
            return
        time.sleep(15)

    print(f"No {args.state} run found after {args.wait}s", file=sys.stderr)
    sys.exit(1)


if __name__ == "__main__":
    main()
