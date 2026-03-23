"""LeRobot Humanoid velocity environment configurations."""

import csv
import math
import re
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
from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
import sys
import torch


_CSV_LOG_STATE_BY_ENV: dict[int, dict[str, Any]] = {}
_FOOT_CONTACT_STATE_BY_ENV: dict[int, dict[str, torch.Tensor]] = {}


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
    self._freq_masks_cache: dict[tuple[int, float, float, str], tuple[torch.Tensor, torch.Tensor]] = {}

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
        "count": torch.zeros(actions.shape[0], device=actions.device, dtype=torch.long),
      }
      self._state_by_env[env_key] = state

    buf = state["buf"]
    pos = int(state["pos"])
    count = state["count"]

    buf[:, pos, :] = actions
    state["pos"] = (pos + 1) % history_len
    count.add_(1).clamp_(max=history_len)

    # Indices of the last `history_len` actions in chronological order.
    idx = torch.tensor(
      [((state["pos"] - history_len + i) % history_len) for i in range(history_len)],
      device=actions.device,
      dtype=torch.long,
    )
    hist = buf.index_select(1, idx)  # [N, T, D]
    # Remove per-window mean so DC offset does not dominate the spectral ratio.
    hist = hist - hist.mean(dim=1, keepdim=True)
    spec = torch.fft.rfft(hist, dim=1)  # [N, F, D]
    power = spec.real.square() + spec.imag.square()
    inband_mask, valid_mask = self._get_freq_masks(history_len, float(env.step_dt), cutoff_hz, actions.device)
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


def _flatten_obs_policy_with_names(
  obs_policy: torch.Tensor | dict[str, torch.Tensor],
  env_index: int,
) -> tuple[list[str], list[float]]:
  """Flatten policy observation with stable CSV column names."""
  if isinstance(obs_policy, torch.Tensor):
    vals = obs_policy[env_index].detach().cpu().reshape(-1).tolist()
    return ([f"obs_{i}" for i in range(len(vals))], vals)

  names: list[str] = []
  values: list[float] = []
  for term_name, term in obs_policy.items():
    term_vals = term[env_index].detach().cpu().reshape(-1).tolist()
    if len(term_vals) == 1:
      names.append(str(term_name))
    else:
      names.extend(f"{term_name}_{i}" for i in range(len(term_vals)))
    values.extend(term_vals)
  return names, values


def _get_projected_gravity_from_policy_obs(
  obs_policy: torch.Tensor | dict[str, torch.Tensor],
  env_index: int,
) -> list[float] | None:
  """Extract projected gravity (x, y, z) when policy obs is a named dict."""
  if not isinstance(obs_policy, dict):
    return None
  gravity = obs_policy.get("projected_gravity")
  if gravity is None:
    return None
  vals = gravity[env_index].detach().cpu().reshape(-1).tolist()
  if len(vals) < 3:
    return None
  return [float(vals[0]), float(vals[1]), float(vals[2])]


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

  obs_names, obs_values = _flatten_obs_policy_with_names(obs_policy, env_index)
  projected_gravity = _get_projected_gravity_from_policy_obs(obs_policy, env_index)
  action_values = (
    env.action_manager.action[env_index].detach().cpu().reshape(-1).tolist()
  )
  action_applied_values = None
  for attr_name in ("processed_action", "applied_action", "command", "action_scaled"):
    attr = getattr(env.action_manager, attr_name, None)
    if isinstance(attr, torch.Tensor):
      try:
        action_applied_values = attr[env_index].detach().cpu().reshape(-1).tolist()
        break
      except Exception:
        continue

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
    header = ["global_step", "episode_step", "env_index"]
    if projected_gravity is not None:
      header += ["projected_gravity_x", "projected_gravity_y", "projected_gravity_z"]
    header += [f"action_{i}" for i in range(len(action_values))]
    if action_applied_values is not None:
      header += [f"action_applied_{i}" for i in range(len(action_applied_values))]
    header += obs_names
    with csv_file.open("w", newline="") as f:
      writer = csv.writer(f)
      writer.writerow(header)
    state["header_written"] = True

  row = [
    int(env.common_step_counter),
    int(env.episode_length_buf[env_index].item()),
    env_index,
  ]
  if projected_gravity is not None:
    row += projected_gravity
  row += action_values
  if action_applied_values is not None:
    row += action_applied_values
  row += obs_values
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


def _get_feet_contact_matrix(
  env,
  sensor_name: str = "feet_ground_contact",
  force_threshold: float | None = 5.0,
  force_contact_threshold: float | None = None,
  force_no_contact_threshold: float | None = None,
  force_threshold_min: float | None = None,
  force_threshold_max: float | None = None,
) -> torch.Tensor | None:
  """Return per-foot contact matrix [num_envs, num_feet] as float32 (0/1)."""
  sensor = None
  sensors = getattr(env.scene, "sensors", None)
  if sensors is not None:
    try:
      sensor = sensors[sensor_name]
    except Exception:
      pass
  if sensor is None:
    try:
      sensor = env.scene[sensor_name]
    except Exception:
      return None

  forces = getattr(sensor.data, "force", None)
  if isinstance(forces, torch.Tensor):
    # Hysteresis thresholds:
    # - force_contact_threshold: switch to contact when force >= threshold.
    # - force_no_contact_threshold: switch to no-contact when force <= threshold.
    # Values in-between keep previous contact state.
    contact_threshold = force_contact_threshold
    if contact_threshold is None:
      contact_threshold = force_threshold_min
    if contact_threshold is None:
      contact_threshold = 0.0 if force_threshold is None else float(force_threshold)

    no_contact_threshold = force_no_contact_threshold
    if no_contact_threshold is None and force_threshold_max is not None and force_threshold_min is None:
      # Backward-compat fallback when only max is provided.
      no_contact_threshold = float(force_threshold_max)
    if no_contact_threshold is None:
      no_contact_threshold = float(contact_threshold)

    contact_threshold = float(contact_threshold)
    no_contact_threshold = float(no_contact_threshold)
    if no_contact_threshold > contact_threshold:
      raise ValueError(
        f"force_no_contact_threshold ({no_contact_threshold}) must be <= "
        f"force_contact_threshold ({contact_threshold}) for hysteresis."
      )

    if forces.ndim >= 3:
      force_mag = torch.linalg.norm(forces, dim=-1)
    else:
      force_mag = torch.abs(forces)
    force_flat = force_mag.reshape(force_mag.shape[0], -1)

    env_key = id(env)
    state_by_sensor = _FOOT_CONTACT_STATE_BY_ENV.get(env_key)
    if state_by_sensor is None:
      state_by_sensor = {}
      _FOOT_CONTACT_STATE_BY_ENV[env_key] = state_by_sensor

    contact_state = state_by_sensor.get(sensor_name)
    needs_init = (
      contact_state is None
      or contact_state.shape != force_flat.shape
      or contact_state.device != force_flat.device
    )
    if needs_init:
      contact_state = torch.zeros(force_flat.shape, dtype=torch.bool, device=force_flat.device)
      state_by_sensor[sensor_name] = contact_state

    # Reset hysteresis memory for just-reset envs.
    episode_length_buf = getattr(env, "episode_length_buf", None)
    if isinstance(episode_length_buf, torch.Tensor) and episode_length_buf.shape[0] == force_flat.shape[0]:
      just_reset = episode_length_buf == 0
      if bool(just_reset.any()):
        contact_state[just_reset] = False

    contact_state = torch.where(
      force_flat >= contact_threshold,
      torch.ones_like(contact_state),
      torch.where(
        force_flat <= no_contact_threshold,
        torch.zeros_like(contact_state),
        contact_state,
      ),
    )
    state_by_sensor[sensor_name] = contact_state
    contact = contact_state.to(dtype=torch.float32)
  else:
    found = getattr(sensor.data, "found", None)
    if not isinstance(found, torch.Tensor):
      return None
    contact = found.to(dtype=torch.float32)

  return contact.reshape(contact.shape[0], -1)


def _single_foot_on_ground_reward(
  env,
  sensor_name: str = "feet_ground_contact",
  force_threshold: float | None = 5.0,
  force_contact_threshold: float | None = None,
  force_no_contact_threshold: float | None = None,
  force_threshold_min: float | None = None,
  force_threshold_max: float | None = None,
) -> torch.Tensor:
  """Reward single-support stance: exactly one foot in contact."""
  contact_flat = _get_feet_contact_matrix(
    env=env,
    sensor_name=sensor_name,
    force_threshold=force_threshold,
    force_contact_threshold=force_contact_threshold,
    force_no_contact_threshold=force_no_contact_threshold,
    force_threshold_min=force_threshold_min,
    force_threshold_max=force_threshold_max,
  )
  if contact_flat is None:
    return torch.zeros(env.num_envs, device=env.action_manager.action.device)
  num_contacts = torch.sum(contact_flat, dim=1)
  return (num_contacts == 1).to(dtype=torch.float32)


class _AlternatingSingleSupportReward:
  """Reward alternating single-support with switch timing near a target frequency."""

  def __init__(self) -> None:
    self._state_by_env: dict[int, dict[str, Any]] = {}
    self._last_env_key: int | None = None

  def __call__(
    self,
    env,
    sensor_name: str = "feet_ground_contact",
    force_threshold: float | None = 5.0,
    force_contact_threshold: float | None = None,
    force_no_contact_threshold: float | None = None,
    force_threshold_min: float | None = None,
    force_threshold_max: float | None = None,
    target_hz: float = 2.0,
    interval_sigma_s: float = 0.08,
  ) -> torch.Tensor:
    contact_flat = _get_feet_contact_matrix(
      env=env,
      sensor_name=sensor_name,
      force_threshold=force_threshold,
      force_contact_threshold=force_contact_threshold,
      force_no_contact_threshold=force_no_contact_threshold,
      force_threshold_min=force_threshold_min,
      force_threshold_max=force_threshold_max,
    )
    device = env.action_manager.action.device
    if contact_flat is None:
      return torch.zeros(env.num_envs, device=device)
    if target_hz <= 0.0:
      raise ValueError(f"target_hz must be > 0, got {target_hz}")

    env_key = id(env)
    self._last_env_key = env_key
    num_envs = contact_flat.shape[0]

    state = self._state_by_env.get(env_key)
    needs_init = (
      state is None
      or state["prev_support_idx"].shape[0] != num_envs
      or state["prev_support_idx"].device != device
    )
    if needs_init:
      state = {
        "prev_support_idx": torch.full((num_envs,), -1, dtype=torch.long, device=device),
        "has_prev_support": torch.zeros((num_envs,), dtype=torch.bool, device=device),
        "steps_since_switch": torch.zeros((num_envs,), dtype=torch.long, device=device),
        "has_prev_switch": torch.zeros((num_envs,), dtype=torch.bool, device=device),
      }
      self._state_by_env[env_key] = state

    steps_since_switch = state["steps_since_switch"]
    has_prev_switch = state["has_prev_switch"]
    steps_since_switch.add_(1)

    num_contacts = torch.sum(contact_flat, dim=1)
    single_support = num_contacts == 1
    support_idx = torch.argmax(contact_flat, dim=1)

    prev_support_idx = state["prev_support_idx"]
    has_prev_support = state["has_prev_support"]
    alternated = single_support & has_prev_support & (support_idx != prev_support_idx)

    step_dt = float(env.step_dt)
    target_interval_s = 1.0 / float(target_hz)
    sigma_s = max(float(interval_sigma_s), 1e-6)
    switch_interval_s = steps_since_switch.to(dtype=torch.float32) * step_dt
    timing_score = torch.exp(-0.5 * torch.square((switch_interval_s - target_interval_s) / sigma_s))
    reward = torch.where(
      alternated & has_prev_switch,
      timing_score,
      torch.zeros_like(timing_score),
    )

    prev_support_idx[single_support] = support_idx[single_support]
    has_prev_support[single_support] = True
    has_prev_switch[alternated] = True
    steps_since_switch[alternated] = 0

    return reward

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    if self._last_env_key is None:
      return
    state = self._state_by_env.get(self._last_env_key)
    if state is None:
      return
    prev_support_idx = state["prev_support_idx"]
    has_prev_support = state["has_prev_support"]
    steps_since_switch = state["steps_since_switch"]
    has_prev_switch = state["has_prev_switch"]
    if env_ids is None or isinstance(env_ids, slice):
      prev_support_idx.fill_(-1)
      has_prev_support.zero_()
      steps_since_switch.zero_()
      has_prev_switch.zero_()
      return
    prev_support_idx[env_ids] = -1
    has_prev_support[env_ids] = False
    steps_since_switch[env_ids] = 0
    has_prev_switch[env_ids] = False


_ALTERNATING_SINGLE_SUPPORT_REWARD = _AlternatingSingleSupportReward()


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

  cfg.sim.mujoco.ccd_iterations = 1000
  cfg.sim.contact_sensor_maxmatch = 1000
  cfg.sim.nconmax = 100

  cfg.scene.entities = {"robot": get_lerobot_humanoid_no_arms_robot_cfg()}

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
    primary=ContactMatch(mode="subtree", pattern="torso_mesh", entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern="torso_mesh", entity="robot"),
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

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE

  cfg.viewer.body_name = "torso_mesh"

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
  base_ang_vel_term.noise.n_min = -0.2
  base_ang_vel_term.noise.n_max = 0.2
  projected_gravity_term = policy_obs.terms.get("projected_gravity")
  projected_gravity_term.noise.n_min = -0.025
  projected_gravity_term.noise.n_max = 0.025
  joint_pos_term = policy_obs.terms.get("joint_pos")
  if joint_pos_term is not None and getattr(joint_pos_term, "noise", None) is not None:
    joint_pos_noise_rad = math.radians(2.5)
    joint_pos_term.noise.n_min = -joint_pos_noise_rad
    joint_pos_term.noise.n_max = joint_pos_noise_rad
  joint_vel_term = policy_obs.terms.get("joint_vel")
  joint_vel_term.noise.n_min = -math.radians(10.0)
  joint_vel_term.noise.n_max = math.radians(10.0)

  # Disable velocity/command curricula while keeping terrain_levels curriculum.
  for curriculum_name in list(cfg.curriculum.keys()):
    if curriculum_name not in {
      "terrain_levels",
      "action_rate_weight",
      "action_rate_hipz_hipx_weight",
    }:
      cfg.curriculum.pop(curriculum_name, None)
  cfg.observations["critic"].terms["foot_height"].params[
    "asset_cfg"
  ].site_names = site_names

  # Disable foot-ground contact randomization for stability/debug.
  # cfg.events.pop("foot_friction", None)
  # Randomize total robot weight by scaling all body masses together (+/-10%).
  cfg.events["robot_weight"] = EventTermCfg(
    func=envs_mdp.randomize_field,
    mode="startup",
    domain_randomization=True,
    params={
      "asset_cfg": SceneEntityCfg(name="robot"),
      "field": "body_mass",
      "operation": "scale",
      "ranges": (0.7, 1.3),
      # Randomize each body independently (not a single global scale).
      "shared_random": False,
    },
  )
  # Randomize joint Coulomb friction per-joint.
  cfg.events["joint_coulomb_friction"] = EventTermCfg(
    func=envs_mdp.randomize_field,
    mode="startup",
    domain_randomization=True,
    params={
      "asset_cfg": SceneEntityCfg(name="robot"),
      "field": "dof_frictionloss",
      "operation": "scale",
      "ranges": (0.5, 1.5),
      "shared_random": False,
    },
  )
  # Randomize joint viscous friction (damping) per-joint.
  cfg.events["joint_viscous_friction"] = EventTermCfg(
    func=envs_mdp.randomize_field,
    mode="startup",
    domain_randomization=True,
    params={
      "asset_cfg": SceneEntityCfg(name="robot"),
      "field": "dof_damping",
      "operation": "scale",
      "ranges": (0.7, 1.3),
      "shared_random": False,
    },
  )
  # Randomize COM placement on all body segments (per-body, not shared).
  cfg.events["base_com"].params["asset_cfg"] = SceneEntityCfg(name="robot")
  cfg.events["base_com"].params["ranges"] = {
    0: (-0.02, 0.02),
    1: (-0.02, 0.02),
    2: (-0.02, 0.02),
  }
  # Simulate encoder calibration mismatch up to +/-5 deg.
  if "encoder_bias" in cfg.events:
    encoder_bias_rad = math.radians(8.0)
    cfg.events["encoder_bias"].params["bias_range"] = (-encoder_bias_rad, encoder_bias_rad)
  # Add reset-time base attitude bias (e.g. IMU mounting / calibration mismatch proxy).
  if "reset_base" in cfg.events:
    reset_pose_range = cfg.events["reset_base"].params.setdefault("pose_range", {})
    tilt_bias_rad = math.radians(2.0)
    reset_pose_range["roll"] = (-tilt_bias_rad, tilt_bias_rad)
    reset_pose_range["pitch"] = (-tilt_bias_rad, tilt_bias_rad)

  # Pose reward std values for the 12-DOF humanoid.
  # The variable_posture reward targets the robot default joint pose
  # (asset.data.default_joint_pos), which comes from KNEES_BENT_KEYFRAME.
  # Hip joints get more freedom, ankle roll is tight for balance.
  cfg.rewards["pose"].params["std_standing"] = {
    # Lower body - 12 DOF.
    r".*hipy.*": 0.8,
    r".*hipx.*": 0.1,
    r".*hipz.*": 0.1,
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
  # Increase posture/upright shaping to keep the torso stable around the
  # nominal standing configuration.
  cfg.rewards["pose"].weight = 2.5

  cfg.rewards["upright"].params["asset_cfg"].body_names = ("torso_mesh",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("torso_mesh",)
  cfg.rewards["upright"].weight = 1.5

  for reward_name in ["foot_clearance", "foot_swing_height", "foot_slip"]:
    cfg.rewards[reward_name].params["asset_cfg"].site_names = site_names

  # Increase command tracking pressure.
  cfg.rewards["track_linear_velocity"].weight = 6.0
  # cfg.rewards["track_angular_velocity"].weight = 4.0

  cfg.rewards["body_ang_vel"].weight = -0.05
  cfg.rewards["angular_momentum"].weight = -0.02
  cfg.rewards["air_time"].weight = 0.0
  # cfg.rewards["single_foot_contact"] = RewardTermCfg(
  #   func=_single_foot_on_ground_reward,
  #   weight=0.3,
  #   params={
  #     "sensor_name": feet_ground_cfg.name,
  #     "force_contact_threshold": 75.0,
  #     "force_no_contact_threshold": 1.0,
  #   },
  # )
  # cfg.rewards["alternating_single_support"] = RewardTermCfg(
  #   func=_ALTERNATING_SINGLE_SUPPORT_REWARD,
  #   weight=0.4,
  #   params={
  #     "sensor_name": feet_ground_cfg.name,
  #     "force_contact_threshold": 75.0,
  #     "force_no_contact_threshold": 4.0,
  #     "target_hz": 2.0,
  #     "interval_sigma_s": 0.08,
  #   },
  # )

  # Encourage lower overall effort, with an extra penalty on ankle torque demand.
  cfg.rewards["actuator_torque_l2"] = RewardTermCfg(
    func=_all_actuator_torque_l2,
    weight=-2e-4,
  )
  # cfg.rewards["ankle_torque_l2"] = RewardTermCfg(
  #   func=_ankle_actuator_torque_l2,
  #   weight=-30e-4,
  # )
  # cfg.rewards["ankle_power_l1"] = RewardTermCfg(
  #   func=_ankle_actuator_power_l1,
  #   weight=-5e-4,
  # )
  cfg.rewards["ankle_torque_over_5nm_l2"] = RewardTermCfg(
    func=_ankle_torque_above_limit_l2,
    weight=-20e-4,
    params={"limit_nm": 4.0},
  )
  # Reward the fraction of action spectral energy within the <=3 Hz band.
  # cfg.rewards["action_fft_band_le_3hz_ratio"] = RewardTermCfg(
  #   func=_ACTION_FFT_BAND_RATIO_REWARD,
  #   weight=3.0,
  #   params={"history_len": 50, "min_history": 50, "cutoff_hz": 2.5},
  # )
  # Keep the standard action-rate smoothing penalty in addition.
  cfg.rewards["action_rate_l2"].weight = -0.1
  cfg.rewards["action_rate_hipz_hipx_l2"] = RewardTermCfg(
    func=_SELECTIVE_ACTION_RATE_L2_PENALTY,
    weight=-5.0,
    params={"joint_name_patterns": (r".*hipz.*", r".*hipx.*")},
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
        {"step": 15_000 * 24, "weight": -2.0},
        {"step": 20_000 * 24, "weight": -4.0},
        {"step": 25_000 * 24, "weight": -4.0},
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
        {"step": 10_000 * 24, "weight": -20.0},
        {"step": 15_000 * 24, "weight": -40.0},
        {"step": 20_000 * 24, "weight": -60.0},
        {"step": 25_000 * 24, "weight": -70.0},
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

    # Disable observation corruption and domain randomization for play testing.
    # cfg.observations["policy"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    # cfg.events.pop("encoder_bias", None)
    for event_name in list(cfg.events.keys()):
      event_cfg = cfg.events[event_name]
      if bool(getattr(event_cfg, "domain_randomization", False)):
        cfg.events.pop(event_name, None)
    if "reset_base" in cfg.events:
      cfg.events["reset_base"].params["pose_range"] = {}
      cfg.events["reset_base"].params["velocity_range"] = {}

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
    twist_cmd.ranges.lin_vel_x = (-0.8, 0.8)
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
