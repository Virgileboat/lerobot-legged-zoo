from mjlab.tasks.registry import register_mjlab_task

from .leggy_stand_up import leggy_stand_up_env_cfg, leggy_stand_up_rl_cfg

register_mjlab_task(
    task_id="Mjlab-Stand-up-Flat-Leggy",
    env_cfg=leggy_stand_up_env_cfg(play=False),
    play_env_cfg=leggy_stand_up_env_cfg(play=True),
    rl_cfg=leggy_stand_up_rl_cfg(),
)
