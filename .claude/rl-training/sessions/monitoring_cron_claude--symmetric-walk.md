You are the autonomous training loop for session claude/symmetric-walk.
To find your cron ID: call CronList, find the entry whose prompt contains "autonomous training loop for session claude/symmetric-walk". Use that ID when calling CronDelete to self-delete.

STEP 0: Read project context.
- cd /home/vbatto/devel/lerobot-legged-zoo
- Read .claude/rl-training/config.md → extract: task name, training command, metric categories, key metrics, kill threshold, max iterations, decision criteria, evaluation config, notification config, source files, host order.
- WandB project: success-hf/mjlab
- Read logs/sessions/claude--symmetric-walk/session_state.json. If missing or corrupted, notify via bash .claude/rl-training/scripts/notify.sh "Monitor error — session_state.json missing" --branch "claude/symmetric-walk" and exit.
- Extract: phase, goal, branch, host, current_run, wandb_run_path, monitor_count, consecutive_bad, iterations.
- Read .claude/rl-training/hosts/remote/host.md for host details.
- TASK_DIR = .claude/rl-training/tasks/locomotion/
- Read TASK_DIR/monitor_config.md → extract: quality thresholds, decision rules, human feedback tags.

STEP 1: Act based on phase.

=== IF phase = "FINISHED" or "PAUSED" ===
Session is done. Find your cron ID via CronList. Call CronDelete. Exit immediately.

=== IF phase = "ITERATE" ===

Check for iterate_in_progress.lock in logs/sessions/claude--symmetric-walk/:
  If it exists: another iterate agent is already running (spawned by !apply). Exit — do not double-iterate.
  If it does not exist: proceed with ITERATE flow below.

Write the lock: python3 -c "import pathlib; pathlib.Path('logs/sessions/claude--symmetric-walk/iterate_in_progress.lock').write_text('cron')"

Read logs/sessions/claude--symmetric-walk/consumed_modification.json:
  a) EXISTS → user-directed change. Use "notes" field as the implementation spec.
  b) MISSING → autonomous kill. Read run_{previous_run}/result.md and last 2 monitor files for context.
     Read docs/training-learnings.md for known patterns.
     Form a single-hypothesis fix.

Ensure worktree exists:
  Check: ls /home/vbatto/devel/lerobot-legged-zoo/lerobot-legged-zoo-wt-claude--symmetric-walk
  If missing: git worktree add lerobot-legged-zoo-wt-claude--symmetric-walk claude/symmetric-walk
  cd /home/vbatto/devel/lerobot-legged-zoo/lerobot-legged-zoo-wt-claude--symmetric-walk && git pull

Read and implement changes in the worktree:
  - lerobot-legged-zoo-wt-claude--symmetric-walk/training_exemples/lerobot_humanoid_no_arms/__init__.py
  - lerobot-legged-zoo-wt-claude--symmetric-walk/training_exemples/lerobot_humanoid_no_arms/env_cfgs.py
  - lerobot-legged-zoo-wt-claude--symmetric-walk/training_exemples/lerobot_humanoid_no_arms/lerobot_humanoid_no_arms_constants.py
  One hypothesis, one fix. No dead code.

Review and commit:
  git diff — verify every change traces to the spec.
  git add <changed files>
  git commit -m "<what and why>"
  (No git push — training uses local repo directly.)

Write context.md:
  logs/sessions/claude--symmetric-walk/run_{current_run:03d}/context.md

Launch training:
  bash .claude/rl-training/hosts/remote/launch.sh "claude/symmetric-walk" "claude--symmetric-walk" "uv run train Mjlab-Velocity-Flat-LeRobot-Humanoid-no-arms --env.scene.num-envs 4096"
  If exit 1: notify "No host available" --branch "claude/symmetric-walk" → remove lock → exit.

Wait for WandB run:
  uv run .claude/rl-training/scripts/get_latest_run.py success-hf/mjlab --state running --wait 120 --branch claude/symmetric-walk

Update session state:
  uv run .claude/rl-training/scripts/session.py update logs/sessions/claude--symmetric-walk \
    --set phase=MONITOR \
    --set wandb_run_path=<path> \
    --set monitor_count=0 \
    --set consecutive_bad=0

Archive consumed_modification.json if it existed:
  Rename to consumed_modification_run_{previous_run:03d}.json

Notify:
  bash .claude/rl-training/scripts/notify.sh "$(printf 'Training Relaunched — Run %d\nBranch: claude/symmetric-walk\nChange: %s' "$current_run" "$CHANGE_SUMMARY")" --branch "claude/symmetric-walk"

Remove lock:
  python3 -c "import pathlib; pathlib.Path('logs/sessions/claude--symmetric-walk/iterate_in_progress.lock').unlink(missing_ok=True)"

Exit (cron continues — will monitor on next tick now that phase=MONITOR).

=== IF phase = "MONITOR" ===

1. Determine M = monitor_count + 1. Pad to 3 digits for filenames.
   Session dir: logs/sessions/claude--symmetric-walk/
   Run directory: <session_dir>/run_{current_run padded to 3 digits}/
   Previous monitor: run_NNN/monitor_{M-1 padded}.md (if M > 1)

2. Fetch metrics:
   uv run .claude/rl-training/scripts/monitor.py <wandb_run_path> [--previous <prev_monitor>] --categories "Episode_Reward/ Episode_Termination/ Train/" --raw-output run_NNN/raw_metrics_{M padded}.json
   Save output to: run_NNN/monitor_{M padded}.md
   Check exit code: if exit code is 2, training has crashed/been killed — treat as BAD immediately (skip to KILL in step 6).
   If other failure: notify via bash .claude/rl-training/scripts/notify.sh "Monitor error — <error>" --branch "claude/symmetric-walk" and exit.

2b. Compute quality metrics:
    Previous derived: run_NNN/derived_metrics_{M-1 padded}.json (if M > 1)
    uv run .claude/rl-training/tasks/locomotion/monitor_metrics.py run_NNN/raw_metrics_{M padded}.json [--previous <prev_derived>] --config .claude/rl-training/tasks/locomotion/monitor_config.md
    Capture stdout → save to run_NNN/derived_metrics_{M padded}.json
    Capture stderr → append to run_NNN/monitor_{M padded}.md

3. Evaluate policy (mandatory — every tick):
   MUJOCO_GL=egl uv run .claude/rl-training/tasks/locomotion/evaluate_policy.py <wandb_run_path> --output-dir run_NNN/ --config .claude/rl-training/config.md
   This produces eval_report.md, eval_metrics.json, eval_raw_data.json, *.mp4.
   Capture "Video: <path>" line from stdout — this is the video file path.
   If eval fails: notify via bash .claude/rl-training/scripts/notify.sh "Eval error — <error>" --branch "claude/symmetric-walk" and continue without eval data.

3b. Read run_NNN/eval_report.md for behavioral analysis context.

3c. BEHAVIORAL OVERRIDE CHECK (autonomous intervention — runs before DECIDE):
    Only applies when M >= 3 (need at least 3 monitors of data).

    Scan the last 3 derived_metrics files (derived_metrics_{M-2}, {M-1}, {M padded}.json) and
    the last 3 raw_metrics files for Episode_Reward/air_time.

    Declare BEHAVIORAL_OVERRIDE=true if ANY of these hold across all 3 last monitors:
      A) air_time == 0.0 in raw metrics (robot never leaves ground — not walking)
      B) bilateral_bias from eval_report > 0.15 AND quality_score not improving
      C) Human feedback in session_state.json iterations for current_run contains
         tags/notes mentioning "not walking", "one leg", "standing", "asymmetric"
         AND same issue persists in eval_report

    IF BEHAVIORAL_OVERRIDE=true:
    - Diagnose the root cause from available data (metrics trends, curriculum values,
      eval_report, human feedback). Be specific: which reward/curriculum is causing it.
    - Propose a concrete fix (e.g. remove a curriculum term, adjust a weight).
    - Write the fix as consumed_modification.json:
      python3 -c "
      import json, pathlib
      entry = {
        'run': <current_run>,
        'summary': '<one-line summary of fix>',
        'parameters': ['<affected params>'],
        'urgency': 'immediate',
        'notes': '<detailed description of what to change and why>'
      }
      pathlib.Path('logs/sessions/claude--symmetric-walk/consumed_modification.json').write_text(json.dumps([entry], indent=2))
      "
    - Notify via Discord:
      bash .claude/rl-training/scripts/notify.sh "$(printf 'Behavioral override triggered (M=%d)\nIssue: <diagnosis>\nFix: <proposed change>\nRelaunching...' M)" --branch "claude/symmetric-walk"
    - Treat as should_kill=true: proceed directly to KILL flow in step 6,
      using the behavioral diagnosis as the "reasons" string.
      Skip steps 4 and 5.

4. Send pre-decision notification (monitor_update event):
   Compose message from quality metrics (step 2b) and eval summary (from eval_report.md Summary section).
   If VIDEO_PATH is non-empty and file exists:
     bash .claude/rl-training/scripts/notify.sh "$MSG" --branch "claude/symmetric-walk" --file "$VIDEO_PATH"
   Else:
     bash .claude/rl-training/scripts/notify.sh "$MSG" --branch "claude/symmetric-walk"

5. DECIDE:
   uv run .claude/rl-training/scripts/decide.py run_NNN/ --monitor M --config .claude/rl-training/config.md --session-dir <session_dir> --task-config .claude/rl-training/tasks/locomotion/monitor_config.md
   Read JSON output: {decision, should_kill, reasons, consecutive_bad, notification}.

6. ACT based on decide.py output:

   If decision = KEEP (not should_kill):
   uv run .claude/rl-training/scripts/session.py update <session_dir> --set monitor_count=M --set consecutive_bad=0

   If decision = BAD (not should_kill):
   uv run .claude/rl-training/scripts/session.py update <session_dir> --set monitor_count=M --set consecutive_bad=<value from decide.py>

   If should_kill = true → KILL:
   - bash .claude/rl-training/hosts/remote/kill.sh claude--symmetric-walk
   - uv run .claude/rl-training/scripts/generate_result.py run_NNN/ --monitor-count M --reason "<reasons from decide.py>" --goal "<goal>"
   - uv run .claude/rl-training/scripts/session.py update <session_dir> --set phase=ITERATE --set current_run=<N+1>
   - uv run .claude/rl-training/scripts/session.py add-iteration <session_dir> --run N --result "<one-line from decide.py reasons>"
   - mkdir -p <session_dir>/run_{new N padded}/
   - If current_run > max_iterations from config: use --set phase=PAUSED instead of ITERATE.
     When PAUSED: also update docs/training-learnings.md with insights from this run, commit.
     Then find your cron ID via CronList. Call CronDelete. Exit.
   - bash .claude/rl-training/scripts/notify.sh "<notification from decide.py>" --branch "claude/symmetric-walk"
   - Exit (do NOT delete cron — it will handle ITERATE on next tick, unless !apply triggers it sooner).

   If decision = FINISH:
   - uv run .claude/rl-training/scripts/session.py update <session_dir> --set phase=FINISHED
   - Update docs/training-learnings.md with insights from this successful run, commit.
   - bash .claude/rl-training/scripts/notify.sh "<notification from decide.py>" --branch "claude/symmetric-walk"
   - Find your cron ID via CronList. Call CronDelete. Exit.

IMPORTANT:
- Always cd to /home/vbatto/devel/lerobot-legged-zoo before running uv commands
- Always use --branch "claude/symmetric-walk" when calling notify.sh
- Always use printf or $'...' to compose multi-line messages, never literal \n strings
- This cron handles all phases. It only deletes itself on FINISH or PAUSED.
