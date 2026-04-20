You are the keepalive agent for training session claude/symmetric-walk.
Your roles: (1) handle ITERATE phase automatically, (2) supervise the monitoring cron during MONITOR phase.

FIRST: Find your own cron ID by calling CronList and identifying the job whose prompt contains "keepalive agent for training session claude/symmetric-walk". Save it — use it if you ever need to call CronDelete to remove yourself.

STEP 0: Read context.
- cd /home/vbatto/devel/lerobot-legged-zoo
- Read .claude/rl-training/config.md → extract training command, source files, host order, max_iterations.
- WandB project: success-hf/mjlab
- Read logs/sessions/claude--symmetric-walk/session_state.json
- Extract: phase, goal, branch, host, current_run, wandb_run_path, monitor_count, iterations.

STEP 1: Act based on phase.

=== phase = FINISHED or PAUSED ===
Session is done. Notify if phase just changed (check if result.md exists in last run dir).
Delete this cron: CronDelete with your ID. Exit.

=== phase = MONITOR ===
Supervise the monitoring cron.
Call CronList, look for entry whose prompt contains "autonomous training loop for session claude/symmetric-walk".
If NOT found:
  - Read .claude/rl-training/sessions/monitoring_cron_claude--symmetric-walk.md
  - CronCreate: cron="*/90 * * * *", recurring=true, durable=true, prompt=<full contents of that file>
  - bash .claude/rl-training/scripts/notify.sh "$(printf 'Monitoring cron recreated\nBranch: claude/symmetric-walk\nRun: %d' "$current_run")" --branch "claude/symmetric-walk"
Exit (nothing else to do in MONITOR phase).

=== phase = ITERATE ===

STEP 2: Determine what changes to implement.

Check logs/sessions/claude--symmetric-walk/consumed_modification.json:
  a) EXISTS → user-directed change. Read it: use the "notes" field as the implementation spec.
     The summary is the one-line description. Urgency is already "immediate" (it was consumed).
  b) MISSING → autonomous kill (monitoring cron or behavioral override killed training).
     Read logs/sessions/claude--symmetric-walk/run_{previous_run padded to 3}/result.md for kill reason.
     Read the last 2 monitor files and last derived_metrics for diagnostic context.
     Read docs/training-learnings.md for known patterns.
     Form a single-hypothesis fix based on the kill reason.

STEP 3: Ensure worktree exists.
  Check if ../lerobot-legged-zoo-wt-claude--symmetric-walk exists.
  If not: git worktree add ../lerobot-legged-zoo-wt-claude--symmetric-walk claude/symmetric-walk
  cd ../lerobot-legged-zoo-wt-claude--symmetric-walk && git pull

STEP 4: Implement changes.
  Read source files listed in config.md → Source Files (work in worktree):
  - training_exemples/lerobot_humanoid_no_arms/__init__.py
  - training_exemples/lerobot_humanoid_no_arms/env_cfgs.py
  - training_exemples/lerobot_humanoid_no_arms/lerobot_humanoid_no_arms_constants.py

  Apply the changes from step 2. Be targeted — one hypothesis, one fix. No dead code.

STEP 5: Review and commit.
  cd ../lerobot-legged-zoo-wt-claude--symmetric-walk
  git diff  — verify every change traces to the fix spec.
  git add <changed files>
  git commit -m "<what and why>"
  git push origin HEAD

STEP 6: Write context.md.
  Path: logs/sessions/claude--symmetric-walk/run_{current_run padded to 3}/context.md
  Document: what was changed, why, which files modified, which fix hypothesis.

STEP 7: Launch training.
  bash .claude/rl-training/hosts/remote/launch.sh "claude/symmetric-walk" "claude--symmetric-walk" "uv run train Mjlab-Velocity-Flat-LeRobot-Humanoid-no-arms --env.scene.num-envs 4096"
  If exit 0: host accepted. If exit 1: notify "No host available" and exit without changing phase.

STEP 8: Wait for WandB run.
  uv run .claude/rl-training/scripts/get_latest_run.py success-hf/mjlab --state running --wait 120 --branch claude/symmetric-walk
  Capture the run path.

STEP 9: Update session state.
  uv run .claude/rl-training/scripts/session.py update logs/sessions/claude--symmetric-walk \
    --set phase=MONITOR \
    --set wandb_run_path=<path from step 8> \
    --set monitor_count=0 \
    --set consecutive_bad=0

STEP 10: Create new monitoring cron.
  Read .claude/rl-training/sessions/monitoring_cron_claude--symmetric-walk.md
  CronCreate: cron="*/90 * * * *", recurring=true, durable=true, prompt=<full contents of that file>

STEP 11: Clean up consumed_modification.json if it existed.
  If consumed_modification.json existed: rename it to consumed_modification_run_{previous_run}.json (archive it).

STEP 12: Notify.
  CHANGE_SUMMARY=<consumed_modification summary OR one-line fix hypothesis>
  bash .claude/rl-training/scripts/notify.sh "$(printf 'Training Relaunched — Run %d\nBranch: claude/symmetric-walk\nHost: remote\nChange: %s' "$current_run" "$CHANGE_SUMMARY")" --branch "claude/symmetric-walk"

IMPORTANT:
- Always cd to /home/vbatto/devel/lerobot-legged-zoo before running scripts
- Always use --branch "claude/symmetric-walk" when calling notify.sh
- Always use printf or $'...' to compose multi-line messages, never literal \n strings
- This cron fires every 13 min. In MONITOR phase it only supervises; in ITERATE phase it implements and relaunches.
