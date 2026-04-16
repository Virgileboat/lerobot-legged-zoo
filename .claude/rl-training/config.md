# RL Training Configuration

## Robot
- Name: lerobot_humanoid_no_arms
- Type: biped
- Specificities: 12 DOF lower-body humanoid (no arms). Position-controlled at 50Hz (4 physics steps × 5ms). Hardware frictionloss is asymmetric per identification (correct nominal values). No base_lin_vel available on real robot IMU. Real actuator latency ~20ms (1 control tick).
- Actuators: [hipz_right, hipz_left, hipx_right, hipx_left, hipy_right, hipy_left, knee_right, knee_left, ankley_right, ankley_left, anklex_right, anklex_left]
- Special mechanics: Action vector alternates right/left per joint type (hipz_r, hipz_l, hipx_r, hipx_l, ...). Action scale = 0.25 × effort_limit / kp. Knee kp=60 per identification — do not change.

## Task
- Name: locomotion
- Simulator: MuJoCo
- Framework: mjlab
- Algorithm: PPO (RSL-RL)
- Objective: Walk with velocity tracking, bilateral symmetric gait, physics-feasible motions (energy below 3Hz) for sim2real transfer

## Training
- Command: uv run train Mjlab-Velocity-Flat-LeRobot-Humanoid-no-arms --env.scene.num-envs 4096
- Execution: local (Claude Code runs on the training machine)
- Env count: 4096

## Hosts
Order: [remote]

## Monitoring
- Tool: wandb
- Task monitoring: locomotion
- Metric categories: [Episode_Reward/, Episode_Termination/, Train/]
- Key metrics: [Train/mean_reward, Episode_Reward/track_linear_velocity, Episode_Reward/action_fft_band_le_3hz_ratio, Episode_Reward/action_rate_hipz_hipx_l2]
- Kill threshold: 2
- Max iterations: 10

## Evaluation
- Scenarios:
  - forward_slow: vx=0.3, vy=0.0, wz=0.0
  - forward_fast: vx=0.8, vy=0.0, wz=0.0
  - lateral: vx=0.0, vy=0.4, wz=0.0
  - combined: vx=0.5, vy=0.2, wz=0.1
- Video: true

## Decision Criteria
- KEEP: reward improving or stable, quality score >= 0.4
- BAD: reward degraded >10%, quality score < 0.4, or quality declining while reward improves for 2+ monitors
- FINISH: reward plateaued (<2% variation over 3 monitors) and quality score >= 0.7

## Notifications
- Enabled: true
- Method: discord
- When: [training_started, monitor_update, training_killed, blocker]

## Source Files
- Task config: training_exemples/lerobot_humanoid_no_arms/__init__.py
- Rewards: training_exemples/lerobot_humanoid_no_arms/env_cfgs.py
- Observations: training_exemples/lerobot_humanoid_no_arms/env_cfgs.py
- Constants: training_exemples/lerobot_humanoid_no_arms/lerobot_humanoid_no_arms_constants.py
