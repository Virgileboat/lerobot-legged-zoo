from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import (
  leggy_flat_env_cfg,
  leggy_rough_env_cfg,
)
from .rl_cfg import leggy_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-Leggy",
  env_cfg=leggy_rough_env_cfg(),
  play_env_cfg=leggy_rough_env_cfg(play=True),
  rl_cfg=leggy_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-Leggy",
  env_cfg=leggy_flat_env_cfg(),
  play_env_cfg=leggy_flat_env_cfg(play=True),
  rl_cfg=leggy_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
