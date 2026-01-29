from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import (
  open_duck_v2_flat_env_cfg,
  open_duck_v2_rough_env_cfg,
)
from .rl_cfg import open_duck_v2_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-Open-Duck-v2",
  env_cfg=open_duck_v2_rough_env_cfg(),
  play_env_cfg=open_duck_v2_rough_env_cfg(play=True),
  rl_cfg=open_duck_v2_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-Open-Duck-v2",
  env_cfg=open_duck_v2_flat_env_cfg(),
  play_env_cfg=open_duck_v2_flat_env_cfg(play=True),
  rl_cfg=open_duck_v2_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
