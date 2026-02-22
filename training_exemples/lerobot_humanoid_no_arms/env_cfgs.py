"""LeRobot Humanoid velocity environment configurations."""

import csv
from datetime import datetime
from pathlib import Path
from typing import Any

from .lerobot_humanoid_no_arms_constants import (
  LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE,
  get_lerobot_humanoid_no_arms_robot_cfg,
)
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
import sys
import torch


_CSV_LOG_STATE_BY_ENV: dict[int, dict[str, Any]] = {}


def _flatten_obs_policy(
  obs_policy: torch.Tensor | dict[str, torch.Tensor],
  env_index: int,
) -> list[float]:
  """Flatten policy observation for one environment into a 1D list."""
  if isinstance(obs_policy, torch.Tensor):
    return obs_policy[env_index].detach().cpu().reshape(-1).tolist()

  flat_values: list[float] = []
  for term in obs_policy.values():
    flat_values.extend(term[env_index].detach().cpu().reshape(-1).tolist())
  return flat_values


def _log_obs_action_csv(
  env,
  env_ids=None,
  env_index: int = 0,
  csv_path: str | None = None,
) -> None:
  """Log policy observation and action to CSV at every environment step."""
  del env_ids  # Unused for global interval events.
  if env_index < 0 or env_index >= env.num_envs:
    return

  obs_dict = env.observation_manager.compute(update_history=False)
  obs_policy = obs_dict.get("policy")
  if obs_policy is None:
    return

  obs_values = _flatten_obs_policy(obs_policy, env_index)
  action_values = (
    env.action_manager.action[env_index].detach().cpu().reshape(-1).tolist()
  )

  env_key = id(env)
  state = _CSV_LOG_STATE_BY_ENV.get(env_key)
  if state is None:
    if csv_path is None:
      stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
      csv_file = (
        Path("logs")
        / "csv"
        / f"lerobot_humanoid_no_arms_flat_play_{stamp}.csv"
      )
    else:
      csv_file = Path(csv_path)
    csv_file.parent.mkdir(parents=True, exist_ok=True)
    state = {"path": csv_file, "header_written": False}
    _CSV_LOG_STATE_BY_ENV[env_key] = state
    print(f"[csv] Logging observations/actions to: {csv_file.resolve()}", flush=True)

  csv_file = state["path"]
  header_written = bool(state["header_written"])

  if not header_written:
    header = (
      ["global_step", "episode_step", "env_index"]
      + [f"action_{i}" for i in range(len(action_values))]
      + [f"obs_{i}" for i in range(len(obs_values))]
    )
    with csv_file.open("w", newline="") as f:
      writer = csv.writer(f)
      writer.writerow(header)
    state["header_written"] = True

  row = (
    [
      int(env.common_step_counter),
      int(env.episode_length_buf[env_index].item()),
      env_index,
    ]
    + action_values
    + obs_values
  )
  with csv_file.open("a", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(row)


def _get_ankle_actuator_ids(asset) -> list[int]:
  """Return actuator indices for ankle joints."""
  actuator_names = getattr(asset, "actuator_names", None)
  if actuator_names is None:
    return []
  return [i for i, name in enumerate(actuator_names) if ("anklex" in name or "ankley" in name)]


def _all_actuator_torque_l2(env) -> torch.Tensor:
  """L2 torque cost over all actuators (per environment)."""
  asset = env.scene["robot"]
  torques = asset.data.actuator_force
  return torch.sum(torch.square(torques), dim=1)


def _ankle_actuator_torque_l2(env) -> torch.Tensor:
  """L2 torque cost over ankle actuators only (per environment)."""
  asset = env.scene["robot"]
  torques = asset.data.actuator_force

  ankle_ids = _get_ankle_actuator_ids(asset)
  if not ankle_ids:
    return torch.zeros(torques.shape[0], device=torques.device, dtype=torques.dtype)

  ankle_torques = torques[:, ankle_ids]
  return torch.sum(torch.square(ankle_torques), dim=1)


def _ankle_actuator_power_l1(env) -> torch.Tensor:
  """L1 ankle mechanical power proxy: sum(|tau * qdot|) per environment."""
  asset = env.scene["robot"]
  torques = asset.data.actuator_force
  ankle_ids = _get_ankle_actuator_ids(asset)
  if not ankle_ids:
    return torch.zeros(torques.shape[0], device=torques.device, dtype=torques.dtype)

  ankle_torques = torques[:, ankle_ids]
  actuator_vel = getattr(asset.data, "actuator_velocity", None)
  if actuator_vel is not None:
    ankle_vel = actuator_vel[:, ankle_ids]
    return torch.sum(torch.abs(ankle_torques * ankle_vel), dim=1)

  # Fallback for backends that don't expose actuator_velocity.
  return torch.sum(torch.abs(ankle_torques), dim=1)


def _ankle_torque_above_limit_l2(env, limit_nm: float = 5.0) -> torch.Tensor:
  """Penalize only torque usage above a soft ankle target limit."""
  asset = env.scene["robot"]
  torques = asset.data.actuator_force
  ankle_ids = _get_ankle_actuator_ids(asset)
  if not ankle_ids:
    return torch.zeros(torques.shape[0], device=torques.device, dtype=torques.dtype)

  ankle_abs = torch.abs(torques[:, ankle_ids])
  over = torch.clamp(ankle_abs - limit_nm, min=0.0)
  return torch.sum(torch.square(over), dim=1)


def _print_actuator_torques(env, env_ids=None) -> None:
  """Print mean/max actuator torque (absolute) for quick debugging during play."""
  asset = env.scene["robot"]
  torques = asset.data.actuator_force
  if env_ids is not None:
    torques = torques[env_ids]
  mean_abs = torques.abs().mean(dim=0).tolist()
  max_abs = torques.abs().max(dim=0).values.tolist()
  names = getattr(asset, "actuator_names", None)
  if names is None:
    names = [f"a{i}" for i in range(len(mean_abs))]
  rows = [
    f"{n}: mean {m:.2f} max {x:.2f}"
    for n, m, x in zip(names, mean_abs, max_abs, strict=False)
  ]
  print("[torque]\n  " + "\n  ".join(rows), flush=True)
  sys.stdout.flush()


def lerobot_humanoid_no_arms_rough_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create LeRobot Humanoid rough terrain velocity configuration."""
  cfg = make_velocity_env_cfg()

  cfg.sim.mujoco.ccd_iterations = 500
  cfg.sim.contact_sensor_maxmatch = 500
  cfg.sim.nconmax = 45

  cfg.scene.entities = {"robot": get_lerobot_humanoid_no_arms_robot_cfg()}

  site_names = ("foot_right", "foot_left")
  geom_names = tuple(
    f"{side}_foot{i}_collision" for side in ("left", "right") for i in range(1, 7)
  )

  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(
      mode="subtree",
      pattern=r"^(foot_subassembly|foot_subassembly_2)$",
      entity="robot",
    ),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  self_collision_cfg = ContactSensorCfg(
    name="self_collision",
    primary=ContactMatch(mode="subtree", pattern="torso_mesh", entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern="torso_mesh", entity="robot"),
    fields=("found",),
    reduce="none",
    num_slots=1,
  )
  cfg.scene.sensors = (feet_ground_cfg, self_collision_cfg)
  
  if cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None:
    cfg.scene.terrain.terrain_generator.curriculum = True

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE

  cfg.viewer.body_name = "torso_mesh"

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.viz.z_offset = 0.9  # Adjust based on robot height.
  # Lower velocity command ranges for training stability.
  twist_cmd.ranges.lin_vel_x = (-0.5, 0.5)
  twist_cmd.ranges.ang_vel_z = (-0.2, 0.2)

  # Disable velocity/command curricula while keeping terrain curriculum.
  for curriculum_name in list(cfg.curriculum.keys()):
    if curriculum_name != "terrain_levels":
      cfg.curriculum.pop(curriculum_name, None)

  cfg.observations["critic"].terms["foot_height"].params[
    "asset_cfg"
  ].site_names = site_names

  cfg.events["foot_friction"].params["asset_cfg"].geom_names = geom_names
  cfg.events["base_com"].params["asset_cfg"].body_names = ("torso_mesh",)

  # Pose reward std values for the 12-DOF humanoid.
  # Hip joints get more freedom, ankle roll is tight for balance.
  cfg.rewards["pose"].params["std_standing"] = {".*": 0.05}
  cfg.rewards["pose"].params["std_walking"] = {
    # Lower body - 12 DOF.
    r".*hipy.*": 0.3,
    r".*hipx.*": 0.15,
    r".*hipz.*": 0.15,
    r".*knee.*": 0.35,
    r".*ankley.*": 0.35,
    r".*anklex.*": 0.2,
  }
  
  cfg.rewards["pose"].params["std_running"] = {
    # Lower body - 12 DOF.
    r".*hipy.*": 0.5,
    r".*hipx.*": 0.2,
    r".*hipz.*": 0.2,
    r".*knee.*": 0.6,
    r".*ankley.*": 0.45,
    r".*anklex.*": 0.25,
  }

  cfg.rewards["upright"].params["asset_cfg"].body_names = ("torso_mesh",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("torso_mesh",)

  for reward_name in ["foot_clearance", "foot_swing_height", "foot_slip"]:
    cfg.rewards[reward_name].params["asset_cfg"].site_names = site_names

  cfg.rewards["body_ang_vel"].weight = -0.05
  cfg.rewards["angular_momentum"].weight = -0.02
  cfg.rewards["air_time"].weight = 0.0

  # Encourage lower overall effort, with an extra penalty on ankle torque demand.
  cfg.rewards["actuator_torque_l2"] = RewardTermCfg(
    func=_all_actuator_torque_l2,
    weight=-2e-4,
  )
  cfg.rewards["ankle_torque_l2"] = RewardTermCfg(
    func=_ankle_actuator_torque_l2,
    weight=-30e-4,
  )
  cfg.rewards["ankle_power_l1"] = RewardTermCfg(
    func=_ankle_actuator_power_l1,
    weight=-5e-4,
  )
  cfg.rewards["ankle_torque_over_5nm_l2"] = RewardTermCfg(
    func=_ankle_torque_above_limit_l2,
    weight=-200e-4,
    params={"limit_nm": 5.0},
  )
  # Replace custom frequency penalty with standard action-rate smoothing penalty.
  cfg.rewards["action_rate_l2"].weight = -0.1

  cfg.rewards["self_collisions"] = RewardTermCfg(
    func=mdp.self_collision_cost,
    weight=-1.0,
    params={"sensor_name": self_collision_cfg.name},
  )
  cfg.scene.terrain.friction = "1.2 0.005 0.0001"
  cfg.scene.terrain.solref = "0.01 1"
  cfg.scene.terrain.solimp = "0.99 0.999 0.001 0.5 2"
  cfg.scene.terrain.contact = "enable"
  # Apply play mode overrides.
  if play:
    # Effectively infinite episode length.
    cfg.episode_length_s = int(1e9)

    cfg.observations["policy"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.events["randomize_terrain"] = EventTermCfg(
      func=envs_mdp.randomize_terrain,
      mode="reset",
      params={},
    )

    if cfg.scene.terrain is not None:
      if cfg.scene.terrain.terrain_generator is not None:
        cfg.scene.terrain.terrain_generator.curriculum = False
        cfg.scene.terrain.terrain_generator.num_cols = 5
        cfg.scene.terrain.terrain_generator.num_rows = 5
        cfg.scene.terrain.terrain_generator.border_width = 10.0

  return cfg


def lerobot_humanoid_no_arms_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create LeRobot Humanoid flat terrain velocity configuration."""
  cfg = lerobot_humanoid_no_arms_rough_env_cfg(play=play)

  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 50
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.nconmax = None

  # Switch to flat terrain.
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  # Disable terrain curriculum.
  assert "terrain_levels" in cfg.curriculum
  del cfg.curriculum["terrain_levels"]

  if play:
    twist_cmd = cfg.commands["twist"]
    assert isinstance(twist_cmd, UniformVelocityCommandCfg)
    twist_cmd.ranges.lin_vel_x = (-0.6, 1.0)
    twist_cmd.ranges.ang_vel_z = (-0.4, 0.4)

    cfg.events["log_obs_action_csv"] = EventTermCfg(
      func=_log_obs_action_csv,
      mode="interval",
      interval_range_s=(0.0, 0.0),
      is_global_time=True,
      params={"env_index": 0},
    )

    cfg.events["print_actuator_torques"] = EventTermCfg(
      func=_print_actuator_torques,
      mode="interval",
      interval_range_s=(0.5, 0.5),
      is_global_time=True,
    )

  return cfg
