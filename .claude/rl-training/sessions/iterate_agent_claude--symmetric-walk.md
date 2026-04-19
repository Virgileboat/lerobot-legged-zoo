You are the iterate agent for session claude/symmetric-walk, spawned immediately by !apply.
Working directory: /home/vbatto/devel/lerobot-legged-zoo

STEP 0: Read context.
- Read logs/sessions/claude--symmetric-walk/session_state.json
- Extract: phase, current_run, goal, branch, host.
- If phase != "ITERATE": print "Phase is not ITERATE — exiting." and stop.

STEP 1: Acquire lock.
Check logs/sessions/claude--symmetric-walk/iterate_in_progress.lock:
  If exists AND contains "cron": the monitoring cron is already handling ITERATE. Exit — do not double-iterate.
  Write lock: python3 -c "import pathlib; pathlib.Path('logs/sessions/claude--symmetric-walk/iterate_in_progress.lock').write_text('apply')"

STEP 2: Determine changes.
Read logs/sessions/claude--symmetric-walk/consumed_modification.json.
Use the "notes" field as the implementation spec. "summary" is the one-line description.
The previous run number = current_run - 1.

STEP 3: Ensure worktree exists.
Check: ls /home/vbatto/devel/lerobot-legged-zoo/lerobot-legged-zoo-wt-claude--symmetric-walk
If missing: cd /home/vbatto/devel/lerobot-legged-zoo && git worktree add lerobot-legged-zoo-wt-claude--symmetric-walk claude/symmetric-walk
Then: cd /home/vbatto/devel/lerobot-legged-zoo/lerobot-legged-zoo-wt-claude--symmetric-walk && git pull

STEP 4: Implement changes.
Read source files in the worktree — all three, even if only one needs changing:
- lerobot-legged-zoo-wt-claude--symmetric-walk/training_exemples/lerobot_humanoid_no_arms/__init__.py
- lerobot-legged-zoo-wt-claude--symmetric-walk/training_exemples/lerobot_humanoid_no_arms/env_cfgs.py
- lerobot-legged-zoo-wt-claude--symmetric-walk/training_exemples/lerobot_humanoid_no_arms/lerobot_humanoid_no_arms_constants.py
Also read docs/training-learnings.md for known pitfalls.

Apply the change from Step 2. One hypothesis, one targeted fix. No dead code.

STEP 5: Review and commit.
cd /home/vbatto/devel/lerobot-legged-zoo/lerobot-legged-zoo-wt-claude--symmetric-walk
git diff — verify every change traces to the spec.
git add <changed files>
git commit -m "<what and why>"
(No git push — training uses local repo directly.)

STEP 6: Write context.md.
Path: /home/vbatto/devel/lerobot-legged-zoo/logs/sessions/claude--symmetric-walk/run_{current_run:03d}/context.md
Document: what was changed, why, which files modified, which hypothesis.

STEP 7: Launch training.
cd /home/vbatto/devel/lerobot-legged-zoo
bash .claude/rl-training/hosts/remote/launch.sh "claude/symmetric-walk" "claude--symmetric-walk" "uv run train Mjlab-Velocity-Flat-LeRobot-Humanoid-no-arms --env.scene.num-envs 4096"
If exit 1: bash .claude/rl-training/scripts/notify.sh "No host available — iterate agent could not relaunch" --branch "claude/symmetric-walk"
          Remove lock. Stop.

STEP 8: Wait for WandB run.
uv run .claude/rl-training/scripts/get_latest_run.py success-hf/mjlab --state running --wait 120 --branch claude/symmetric-walk
Capture the run path.

STEP 9: Update session state.
uv run .claude/rl-training/scripts/session.py update logs/sessions/claude--symmetric-walk \
  --set phase=MONITOR \
  --set wandb_run_path=<path from step 8> \
  --set monitor_count=0 \
  --set consecutive_bad=0

STEP 10: Archive consumed_modification.json.
Rename logs/sessions/claude--symmetric-walk/consumed_modification.json
     to logs/sessions/claude--symmetric-walk/consumed_modification_run_{previous_run:03d}.json

STEP 11: Remove lock.
python3 -c "import pathlib; pathlib.Path('logs/sessions/claude--symmetric-walk/iterate_in_progress.lock').unlink(missing_ok=True)"

STEP 12: Notify.
CHANGE_SUMMARY=<summary from consumed_modification.json>
bash .claude/rl-training/scripts/notify.sh "$(printf 'Training Relaunched — Run %d\nBranch: claude/symmetric-walk\nHost: remote\nChange: %s' "$current_run" "$CHANGE_SUMMARY")" --branch "claude/symmetric-walk"

NOTE: The monitoring cron (every 33 min) will resume monitoring now that phase=MONITOR.
No cron management needed — the single monitoring cron persists through the full session.

IMPORTANT:
- Always cd to /home/vbatto/devel/lerobot-legged-zoo before running uv/bash scripts
- Always use --branch "claude/symmetric-walk" when calling notify.sh
- Always use printf or $'...' to compose multi-line messages
