"""LeRobot Humanoid velocity environment configurations."""

import csv
import json
import math
import re
from datetime import datetime
from pathlib import Path
from typing import Any

from .lerobot_humanoid_no_arms_constants import (
  LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE,
  get_lerobot_humanoid_no_arms_robot_cfg,
)
from mjlab.actuator.delayed_actuator import DelayedActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
import sys
import torch


_CSV_LOG_STATE_BY_ENV: dict[int, dict[str, Any]] = {}


class _SelectiveActionRateL2Penalty:
  """L2 action-rate penalty computed on a selected subset of action dimensions."""

  def __init__(self) -> None:
    self._state_by_env: dict[int, dict[str, Any]] = {}

  def _resolve_joint_indices(
    self,
    env,
    num_actions: int,
    joint_name_patterns: tuple[str, ...],
    device: torch.device,
  ) -> torch.Tensor:
    if num_actions <= 0:
      return torch.empty((0,), dtype=torch.long, device=device)

    asset = env.scene["robot"]
    candidate_name_lists = [
      ("actuator_names", getattr(asset, "actuator_names", None)),
      ("joint_names", getattr(asset, "joint_names", None)),
      ("dof_names", getattr(asset, "dof_names", None)),
    ]
    compiled_patterns = [re.compile(p) for p in joint_name_patterns]

    checked_sources: list[str] = []
    for source_name, names in candidate_name_lists:
      if names is None or len(names) != num_actions:
        size = "None" if names is None else str(len(names))
        checked_sources.append(f"{source_name}={size}")
        continue
      checked_sources.append(f"{source_name}={len(names)}")
      ids = [
        i
        for i, name in enumerate(names)
        if any(pattern.fullmatch(name) for pattern in compiled_patterns)
      ]
      if ids:
        return torch.tensor(ids, dtype=torch.long, device=device)
    raise ValueError(
      "No action dimensions matched joint_name_patterns="
      f"{joint_name_patterns} with num_actions={num_actions}. "
      f"Checked sources: {', '.join(checked_sources)}."
    )

  def __call__(
    self,
    env,
    joint_name_patterns: tuple[str, ...] = (r".*hipz.*", r".*hipx.*"),
  ) -> torch.Tensor:
    actions = env.action_manager.action
    prev_actions = env.action_manager.prev_action
    env_key = id(env)

    state = self._state_by_env.get(env_key)
    needs_init = (
      state is None
      or state["joint_ids"].device != actions.device
      or state["num_actions"] != actions.shape[1]
      or state["joint_name_patterns"] != joint_name_patterns
    )
    if needs_init:
      state = {
        "joint_ids": self._resolve_joint_indices(
          env=env,
          num_actions=actions.shape[1],
          joint_name_patterns=joint_name_patterns,
          device=actions.device,
        ),
        "num_actions": actions.shape[1],
        "joint_name_patterns": joint_name_patterns,
      }
      self._state_by_env[env_key] = state

    joint_ids = state["joint_ids"]

    if joint_ids.numel() == 0:
      return torch.zeros(actions.shape[0], device=actions.device, dtype=actions.dtype)
    return torch.sum(torch.square(actions[:, joint_ids] - prev_actions[:, joint_ids]), dim=1)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    del env_ids


class _ActionFftBandRatioReward:
  """Reward the fraction of action spectral energy inside a low-frequency band."""

  def __init__(self) -> None:
    self._state_by_env: dict[int, dict[str, Any]] = {}
    self._last_env_key: int | None = None
    self._freq_masks_cache: dict[
      tuple[int, float, float, str], tuple[torch.Tensor, torch.Tensor]
    ] = {}

  def _get_freq_masks(
    self,
    history_len: int,
    step_dt: float,
    cutoff_hz: float,
    device: torch.device,
  ) -> tuple[torch.Tensor, torch.Tensor]:
    key = (history_len, round(step_dt, 8), cutoff_hz, str(device))
    masks = self._freq_masks_cache.get(key)
    if masks is not None:
      return masks
    freqs = torch.fft.rfftfreq(history_len, d=step_dt, device=device)
    valid_mask = freqs > 0.0
    inband_mask = (freqs <= cutoff_hz) & valid_mask
    masks = (inband_mask, valid_mask)
    self._freq_masks_cache[key] = masks
    return masks

  def __call__(
    self,
    env,
    history_len: int = 50,
    min_history: int = 50,
    cutoff_hz: float = 3.0,
  ) -> torch.Tensor:
    actions = env.action_manager.action
    if history_len < 4:
      raise ValueError(f"history_len must be >= 4 for FFT reward, got {history_len}")
    env_key = id(env)
    self._last_env_key = env_key

    state = self._state_by_env.get(env_key)
    needs_init = (
      state is None
      or state["buf"].shape[0] != actions.shape[0]
      or state["buf"].shape[2] != actions.shape[1]
      or state["buf"].shape[1] != history_len
    )
    if needs_init:
      state = {
        "buf": torch.zeros(
          (actions.shape[0], history_len, actions.shape[1]),
          device=actions.device,
          dtype=actions.dtype,
        ),
        "pos": 0,
        "count": torch.zeros(
          actions.shape[0], device=actions.device, dtype=torch.long
        ),
      }
      self._state_by_env[env_key] = state

    buf = state["buf"]
    pos = int(state["pos"])
    count = state["count"]

    buf[:, pos, :] = actions
    state["pos"] = (pos + 1) % history_len
    count.add_(1).clamp_(max=history_len)

    idx = torch.tensor(
      [((state["pos"] - history_len + i) % history_len) for i in range(history_len)],
      device=actions.device,
      dtype=torch.long,
    )
    hist = buf.index_select(1, idx)
    hist = hist - hist.mean(dim=1, keepdim=True)
    spec = torch.fft.rfft(hist, dim=1)
    power = spec.real.square() + spec.imag.square()
    inband_mask, valid_mask = self._get_freq_masks(
      history_len, float(env.step_dt), cutoff_hz, actions.device
    )
    if not bool(valid_mask.any()):
      reward = torch.ones(actions.shape[0], device=actions.device, dtype=actions.dtype)
    else:
      total_power = power[:, valid_mask, :].sum(dim=(1, 2))
      inband_power = power[:, inband_mask, :].sum(dim=(1, 2))
      reward = torch.where(
        total_power > 1e-12,
        inband_power / total_power,
        torch.ones_like(total_power),
      )

    if min_history > 0:
      ready = count >= min(min_history, history_len)
      reward = torch.where(ready, reward, torch.zeros_like(reward))

    return reward

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    if self._last_env_key is None:
      return
    state = self._state_by_env.get(self._last_env_key)
    if state is None:
      return
    buf = state["buf"]
    count = state["count"]
    if env_ids is None or isinstance(env_ids, slice):
      buf.zero_()
      count.zero_()
      state["pos"] = 0
      return
    buf[env_ids] = 0.0
    count[env_ids] = 0


_ACTION_FFT_BAND_RATIO_REWARD = _ActionFftBandRatioReward()
_SELECTIVE_ACTION_RATE_L2_PENALTY = _SelectiveActionRateL2Penalty()


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
  """Log policy observation and action to CSV at every environment step.

  Output format matches the real-robot debug CSV so sim and real logs can be
  compared directly:  time_s, step, observation (JSON), action_pre_scale (JSON).
  """
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
        / f"lerobot_humanoid_no_arms_play_{stamp}.csv"
      )
    else:
      csv_file = Path(csv_path)
    csv_file.parent.mkdir(parents=True, exist_ok=True)
    state = {"path": csv_file, "header_written": False}
    _CSV_LOG_STATE_BY_ENV[env_key] = state
    print(f"[csv] Logging observations/actions to: {csv_file.resolve()}", flush=True)

  csv_file = state["path"]

  if not state["header_written"]:
    with csv_file.open("w", newline="") as f:
      csv.writer(f).writerow(["time_s", "step", "observation", "action_pre_scale"])
    state["header_written"] = True

  sim_time = float(env.sim.data.time.item() if hasattr(env.sim.data.time, "item") else env.sim.data.time[0])
  obs_json = json.dumps(obs_values, separators=(",", ":"))
  act_json = json.dumps(action_values, separators=(",", ":"))
  with csv_file.open("a", newline="") as f:
    csv.writer(f).writerow([f"{sim_time:.6f}", int(env.common_step_counter), obs_json, act_json])


def _joint_torques_obs(env) -> torch.Tensor:
  """Return actuator torques for all joints. Shape: [num_envs, 12]."""
  return env.scene["robot"].data.actuator_force


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


def lerobot_humanoid_no_arms_rough_env_cfg(play: bool = False, torque_obs: bool = True) -> ManagerBasedRlEnvCfg:
  """Create LeRobot Humanoid rough terrain velocity configuration."""
  cfg = make_velocity_env_cfg()

  cfg.sim.mujoco.ccd_iterations = 500
  cfg.sim.contact_sensor_maxmatch = 500
  cfg.sim.nconmax = 100

  # Wrap all actuators with randomized delay centred on the measured 1 control-tick
  # latency (20 ms on the real robot, physics dt = 5 ms → 1 tick = 4 physics steps).
  # Randomise from 0 to 2 control ticks (0–8 physics steps) for robustness.
  robot_cfg = get_lerobot_humanoid_no_arms_robot_cfg()
  orig_artic = robot_cfg.articulation
  robot_cfg.articulation = EntityArticulationInfoCfg(
    actuators=tuple(
      DelayedActuatorCfg(base_cfg=a, delay_min_lag=0, delay_max_lag=8)
      for a in orig_artic.actuators
    ),
    soft_joint_pos_limit_factor=orig_artic.soft_joint_pos_limit_factor,
  )
  cfg.scene.entities = {"robot": robot_cfg}

  site_names = ("foot_right", "foot_left")
  geom_names = ("left_foot_collision", "right_foot_collision")

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
    primary=ContactMatch(mode="subtree", pattern="torso_subassembly", entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern="torso_subassembly", entity="robot"),
    fields=("found",),
    reduce="none",
    num_slots=1,
  )
  cfg.scene.sensors = (feet_ground_cfg, self_collision_cfg)

  if cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None:
    cfg.scene.terrain.terrain_generator.curriculum = True
    cfg.scene.terrain.terrain_generator.difficulty_range = (0.0, 0.3)
    if hasattr(cfg.scene.terrain, "max_init_terrain_level"):
      cfg.scene.terrain.max_init_terrain_level = 2

  # Disable default velocity-command curriculum, keep terrain curriculum.
  cfg.curriculum.pop("command_vel", None)

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE

  cfg.viewer.body_name = "torso_subassembly"

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.viz.z_offset = 0.9  # Adjust based on robot height.
  # Lower velocity command ranges for training stability.
  twist_cmd.ranges.lin_vel_x = (-0.8, 0.8)
  twist_cmd.ranges.lin_vel_y = (-0.4, 0.4)
  twist_cmd.ranges.ang_vel_z = (-0.2, 0.2)

  # Stronger observation randomization for sim-to-real robustness.
  policy_obs = cfg.observations["policy"]
  base_lin_vel_term = policy_obs.terms.get("base_lin_vel")
  if base_lin_vel_term is not None and getattr(base_lin_vel_term, "noise", None) is not None:
    base_lin_vel_term.noise.n_min = -0.075
    base_lin_vel_term.noise.n_max = 0.075
  policy_obs.terms.pop("base_lin_vel", None)
  base_ang_vel_term = policy_obs.terms.get("base_ang_vel")
  base_ang_vel_term.noise.n_min = -0.06
  base_ang_vel_term.noise.n_max = 0.06
  projected_gravity_term = policy_obs.terms.get("projected_gravity")
  projected_gravity_term.noise.n_min = -0.015
  projected_gravity_term.noise.n_max = 0.015
  joint_pos_term = policy_obs.terms.get("joint_pos")
  if joint_pos_term is not None and getattr(joint_pos_term, "noise", None) is not None:
    joint_pos_noise_rad = math.radians(4.0)
    joint_pos_term.noise.n_min = -joint_pos_noise_rad
    joint_pos_term.noise.n_max = joint_pos_noise_rad
  joint_vel_term = policy_obs.terms.get("joint_vel")
  joint_vel_term.noise.n_min = -0.1
  joint_vel_term.noise.n_max = 0.1
  # Joint torques with noise (sim2real: real actuators have torque measurement noise).
  if torque_obs:
    policy_obs.terms["joint_torques"] = ObservationTermCfg(
      func=_joint_torques_obs,
      noise=Unoise(n_min=-0.05, n_max=0.05),  # ±0.05 Nm — matches real torque readout noise
      scale=1.0 / 88.0,  # Normalize by a representative effort limit.
    )

  cfg.observations["critic"].terms["foot_height"].params[
    "asset_cfg"
  ].site_names = site_names

  # ---------------------------------------------------------------------------
  # Domain Randomization: Contact Parameters
  # ---------------------------------------------------------------------------
  cfg.events["foot_friction"].params["asset_cfg"].geom_names = geom_names
  cfg.events["foot_friction"].params["ranges"] = (0.35, 1.20)  # geom_friction[0]
  cfg.events["foot_friction"].params["shared_random"] = True
  cfg.events["foot_friction"].domain_randomization = True
  cfg.events["foot_friction_torsional"] = EventTermCfg(
    func=envs_mdp.randomize_field,
    mode="startup",
    domain_randomization=True,
    params={
      "field": "geom_friction",
      "ranges": {1: (0.002, 0.020)},  # geom_friction[1]
      "operation": "abs",
      "asset_cfg": SceneEntityCfg("robot", geom_names=geom_names),
      "shared_random": True,
    },
  )
  cfg.events["foot_friction_rolling"] = EventTermCfg(
    func=envs_mdp.randomize_field,
    mode="startup",
    domain_randomization=True,
    params={
      "field": "geom_friction",
      "ranges": {2: (0.00005, 0.00100)},  # geom_friction[2]
      "operation": "abs",
      "asset_cfg": SceneEntityCfg("robot", geom_names=geom_names),
      "shared_random": True,
    },
  )

  # ---------------------------------------------------------------------------
  # Domain Randomization: Joint Dynamics
  # ---------------------------------------------------------------------------
  joint_groups = {
    "hipz": ("hipz_right", "hipz_left"),
    "hipx": ("hipx_right", "hipx_left"),
    "hipy": ("hipy_right", "hipy_left"),
    "knee": ("knee_right", "knee_left"),
    "ankley": ("ankley_right", "ankley_left"),
    "anklex": ("anklex_right", "anklex_left"),
  }
  joint_armature_scales = {
    "hipz": (0.85, 1.15),
    "hipx": (0.85, 1.15),
    "hipy": (0.90, 1.10),
    "knee": (0.90, 1.10),
    "ankley": (0.80, 1.20),
    "anklex": (0.80, 1.20),
  }
  joint_damping_scales = {
    "hipz": (0.70, 1.40),
    "hipx": (0.70, 1.40),
    "hipy": (0.80, 1.30),
    "knee": (0.80, 1.30),
    "ankley": (0.60, 1.80),
    "anklex": (0.60, 1.80),
  }
  # "Static friction" approximation at the joint level (MuJoCo frictionloss).
  joint_frictionloss_scales = {
    "hipz": (0.60, 1.60),
    "hipx": (0.60, 1.60),
    "hipy": (0.70, 1.50),
    "knee": (0.70, 1.50),
    "ankley": (0.50, 2.00),
    "anklex": (0.50, 2.00),
  }
  for group_name, joint_names in joint_groups.items():
    asset_cfg = SceneEntityCfg("robot", joint_names=list(joint_names))
    cfg.events[f"joint_armature_{group_name}"] = EventTermCfg(
      func=envs_mdp.randomize_field,
      mode="startup",
      domain_randomization=True,
      params={
        "field": "dof_armature",
        "ranges": joint_armature_scales[group_name],
        "operation": "scale",
        "asset_cfg": asset_cfg,
      },
    )
    cfg.events[f"joint_damping_{group_name}"] = EventTermCfg(
      func=envs_mdp.randomize_field,
      mode="startup",
      domain_randomization=True,
      params={
        "field": "dof_damping",
        "ranges": joint_damping_scales[group_name],
        "operation": "scale",
        "asset_cfg": asset_cfg,
      },
    )
    cfg.events[f"joint_frictionloss_{group_name}"] = EventTermCfg(
      func=envs_mdp.randomize_field,
      mode="startup",
      domain_randomization=True,
      params={
        "field": "dof_frictionloss",
        "ranges": joint_frictionloss_scales[group_name],
        "operation": "scale",
        "asset_cfg": asset_cfg,
      },
    )

  # ---------------------------------------------------------------------------
  # Domain Randomization: Body Inertial Parameters (Torso + Limbs)
  # ---------------------------------------------------------------------------
  dr_body_names = (
    "torso_subassembly",
    "hipx_subassembly",
    "hipy_subassembly",
    "hipx_subassemby_sym",
    "hipy_subassembly_sym",
    "tigh_subassembly",
    "tigh_subassembly_sym",
    "shin_subassembly",
    "shin_subassembly_sym",
    "ankle_subassembly",
    "ankle_subassembly_2",
    "foot_subassembly",
    "foot_subassembly_2",
  )

  # COM: +/- 3 cm per body on x/y/z.
  cfg.events["base_com"].params["asset_cfg"].body_names = dr_body_names
  cfg.events["base_com"].params["ranges"] = {
    0: (-0.03, 0.03),
    1: (-0.03, 0.03),
    2: (-0.03, 0.03),
  }
  cfg.events["base_com"].domain_randomization = True

  # Mass: +/- 15% per body.
  cfg.events["body_mass"] = EventTermCfg(
    func=envs_mdp.randomize_field,
    mode="startup",
    domain_randomization=True,
    params={
      "field": "body_mass",
      "ranges": (0.85, 1.15),
      "operation": "scale",
      "asset_cfg": SceneEntityCfg("robot", body_names=dr_body_names),
    },
  )
  cfg.events["body_inertia"] = EventTermCfg(
    func=envs_mdp.randomize_field,
    mode="startup",
    domain_randomization=True,
    params={
      "field": "body_inertia",
      "ranges": (0.85, 1.15),
      "operation": "scale",
      "asset_cfg": SceneEntityCfg("robot", body_names=dr_body_names),
    },
  )

  # Remove legacy no-arms-specific DR terms that were tied to older pipelines.
  for event_name in (
    "robot_weight",
    "joint_coulomb_friction",
    "joint_viscous_friction",
    "joint_armature",
  ):
    cfg.events.pop(event_name, None)

  # Pose reward std values for the 12-DOF humanoid.
  # The variable_posture reward targets the robot default joint pose
  # (asset.data.default_joint_pos), which comes from KNEES_BENT_KEYFRAME.
  # Hip joints get more freedom, ankle roll is tight for balance.
  cfg.rewards["pose"].params["std_standing"] = {
    # Lower body - 12 DOF.
    # Tighter hipz/hipx stds encourage stillness when command velocity is zero.
    r".*hipy.*": 0.8,
    r".*hipx.*": 0.05,
    r".*hipz.*": 0.05,
    r".*knee.*": 0.8,
    r".*ankley.*": 0.35,
    r".*anklex.*": 0.2,
  }

  cfg.rewards["pose"].params["std_walking"] = {
    # Lower body - 12 DOF.
    r".*hipy.*": 0.8,
    r".*hipx.*": 0.1,
    r".*hipz.*": 0.1,
    r".*knee.*": 0.8,
    r".*ankley.*": 0.35,
    r".*anklex.*": 0.2,
  }

  cfg.rewards["pose"].params["std_running"] = {
    # Lower body - 12 DOF.
    r".*hipy.*": 0.8,
    r".*hipx.*": 0.1,
    r".*hipz.*": 0.1,
    r".*knee.*": 0.8,
    r".*ankley.*": 0.2,
    r".*anklex.*": 0.1,
  }
  # Match G1 reward parametrization while keeping robot-specific pose std maps.
  cfg.rewards["pose"].weight = 1.0

  cfg.rewards["upright"].params["asset_cfg"].body_names = ("torso_subassembly",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("torso_subassembly",)
  cfg.rewards["upright"].weight = 1.0

  for reward_name in ["foot_clearance", "foot_swing_height", "foot_slip"]:
    cfg.rewards[reward_name].params["asset_cfg"].site_names = site_names

  cfg.rewards["track_linear_velocity"].weight = 2.0
  cfg.rewards["track_angular_velocity"].weight = 2.0

  cfg.rewards["body_ang_vel"].weight = -0.05
  cfg.rewards["dof_pos_limits"].weight = -1.0
  cfg.rewards["angular_momentum"].weight = -0.02
  cfg.rewards["air_time"].weight = 0.0
  cfg.rewards["foot_slip"].weight = -0.1
  cfg.rewards["soft_landing"].weight = -1e-5

  # Re-enable FFT smoothness shaping used in prior no-arms tuning.
  cfg.rewards["action_fft_band_le_3hz_ratio"] = RewardTermCfg(
    func=_ACTION_FFT_BAND_RATIO_REWARD,
    weight=3.0,
    params={"history_len": 50, "min_history": 50, "cutoff_hz": 2.5},
  )
  cfg.rewards["action_rate_l2"].weight = -0.1
  cfg.rewards["action_rate_hipz_hipx_l2"] = RewardTermCfg(
    func=_SELECTIVE_ACTION_RATE_L2_PENALTY,
    weight=-5.0,
    params={"joint_name_patterns": (r".*hipz.*", r".*hipx.*")},
  )
  cfg.curriculum["pose_weight"] = CurriculumTermCfg(
    func=mdp.reward_weight,
    params={
      "reward_name": "pose",
      "weight_stages": [
        {"step": 0, "weight": 2.5},
        # Curriculum uses env.common_step_counter (env steps), while W&B "Step"
        # is PPO iterations. Here num_steps_per_env=24, so multiply by 24.
        {"step": 2_000 * 24, "weight": 2.0},
        {"step": 4_000 * 24, "weight": 1.5},
        {"step": 5_000 * 24, "weight": 1.5},
      ],
    },
  )
  cfg.curriculum["action_rate_weight"] = CurriculumTermCfg(
    func=mdp.reward_weight,
    params={
      "reward_name": "action_rate_l2",
      "weight_stages": [
        {"step": 0, "weight": -0.1},
        # Curriculum uses env.common_step_counter (env steps), while W&B "Step"
        # is PPO iterations. Here num_steps_per_env=24, so multiply by 24.
        {"step": 5_000 * 24, "weight": -0.5},
        {"step": 10_000 * 24, "weight": -1.0},
        {"step": 15_000 * 24, "weight": -1.5},
        {"step": 20_000 * 24, "weight": -2.0},
        {"step": 25_000 * 24, "weight": -2.5},
        {"step": 30_000 * 24, "weight": -4.0},
      ],
    },
  )
  cfg.curriculum["action_rate_hipz_hipx_weight"] = CurriculumTermCfg(
    func=mdp.reward_weight,
    params={
      "reward_name": "action_rate_hipz_hipx_l2",
      "weight_stages": [
        {"step": 0, "weight": -5.0},
        # Curriculum uses env.common_step_counter (env steps), while W&B "Step"
        # is PPO iterations. Here num_steps_per_env=24, so multiply by 24.
        {"step": 5_000 * 24, "weight": -10.0},
        {"step": 10_000 * 24, "weight": -15.0},
        {"step": 15_000 * 24, "weight": -20.0},
        {"step": 20_000 * 24, "weight": -30.0},
        {"step": 25_000 * 24, "weight": -30.0},
      ],
    },
  )

  cfg.rewards["self_collisions"] = RewardTermCfg(
    func=mdp.self_collision_cost,
    weight=-1.0,
    params={"sensor_name": self_collision_cfg.name},
  )
  # Terrain-side stiff contact settings to limit foot penetration.
  # Keep a fixed friction triplet to reduce contact jitter with mesh feet.
  cfg.scene.terrain.friction = "1.0 0.005 0.0001"
  cfg.scene.terrain.solref = "0.005 1"
  cfg.scene.terrain.solimp = "0.995 0.9995 0.001 0.5 2"
  cfg.scene.terrain.contact = "enable"
  # Apply play mode overrides.
  if play:
    # Effectively infinite episode length.
    cfg.episode_length_s = int(1e9)

    cfg.viewer.azimuth = 135.0
    cfg.viewer.elevation = -20.0

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

    # cfg.events["log_obs_action_csv"] = EventTermCfg(
    #   func=_log_obs_action_csv,
    #   mode="interval",
    #   interval_range_s=(0.0, 0.0),
    #   is_global_time=True,
    #   params={"env_index": 0},
    # )

  return cfg


def lerobot_humanoid_no_arms_flat_env_cfg(play: bool = False, torque_obs: bool = True) -> ManagerBasedRlEnvCfg:
  """Create LeRobot Humanoid flat terrain velocity configuration."""
  cfg = lerobot_humanoid_no_arms_rough_env_cfg(play=play, torque_obs=torque_obs)

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
    twist_cmd.ranges.lin_vel_x = (0.3, 0.8)
    twist_cmd.ranges.lin_vel_y = (0.0, 0.0)
    twist_cmd.ranges.ang_vel_z = (0.0, 0.0)

    # cfg.events["print_actuator_torques"] = EventTermCfg(
    #   func=_print_actuator_torques,
    #   mode="interval",
    #   interval_range_s=(0.5, 0.5),
    #   is_global_time=True,
    # )

  return cfg
