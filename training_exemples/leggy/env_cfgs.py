"""Leggy velocity environment configurations."""

from .leggy_actions import LeggyJointActionCfg, joint_pos_motor, joint_vel_motor
from .leggy_constants import get_leggy_robot_cfg
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg


def leggy_rough_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create Leggy rough terrain velocity configuration."""
  cfg = make_velocity_env_cfg()

  cfg.sim.mujoco.ccd_iterations = 500
  cfg.sim.contact_sensor_maxmatch = 500
  cfg.sim.nconmax = 45

  cfg.scene.entities = {"robot": get_leggy_robot_cfg()}

  site_names = ("left_foot", "right_foot")
  geom_names = ("left_foot_collision", "right_foot_collision")

  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(mode="geom", pattern=geom_names, entity="robot"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  cfg.scene.sensors = (feet_ground_cfg,)

  if cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None:
    cfg.scene.terrain.terrain_generator.curriculum = True
    cfg.scene.terrain.terrain_generator.difficulty_range = (0.0, 0.3)
    for sub_cfg in cfg.scene.terrain.terrain_generator.sub_terrains.values():
      if hasattr(sub_cfg, "step_height_range"):
        sub_cfg.step_height_range = (0.0, 0.03)
      if hasattr(sub_cfg, "slope_range"):
        sub_cfg.slope_range = (0.0, 0.3)
      if hasattr(sub_cfg, "noise_range"):
        sub_cfg.noise_range = (0.01, 0.03)
      if hasattr(sub_cfg, "amplitude_range"):
        sub_cfg.amplitude_range = (0.0, 0.05)

  cfg.actions = {"joint_pos": LeggyJointActionCfg()}

  cfg.viewer.body_name = "boddy"

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.viz.z_offset = 0.3
  twist_cmd.ranges.lin_vel_x = (-0.15, 0.15)
  twist_cmd.ranges.lin_vel_y = (-0.08, 0.08)
  twist_cmd.ranges.ang_vel_z = (-0.15, 0.15)

  cfg.observations["critic"].terms["foot_height"].params[
    "asset_cfg"
  ].site_names = site_names

  cfg.events["foot_friction"].params["asset_cfg"].geom_names = geom_names
  cfg.events["base_com"].params["asset_cfg"].body_names = ("boddy",)

  # Pose reward std values for Leggy's actuated joints only.
  cfg.rewards["pose"].params["asset_cfg"].joint_names = (
    "LhipY",
    "LhipX",
    "Lknee",
    "RhipY",
    "RhipX",
    "Rknee",
  )
  cfg.rewards["pose"].params["std_standing"] = {
    "LhipY": 0.2,
    "LhipX": 0.4,
    "Lknee": 0.4,
    "RhipY": 0.2,
    "RhipX": 0.4,
    "Rknee": 0.4,
  }
  cfg.rewards["pose"].params["std_walking"] = {
    "LhipY": 0.2,
    "LhipX": 0.6,
    "Lknee": 0.8,
    "RhipY": 0.2,
    "RhipX": 0.6,
    "Rknee": 0.8,
  }
  cfg.rewards["pose"].params["std_running"] = {
    "LhipY": 0.2,
    "LhipX": 0.5,
    "Lknee": 0.6,
    "RhipY": 0.2,
    "RhipX": 0.5,
    "Rknee": 0.6,
  }

  cfg.rewards["upright"].params["asset_cfg"].body_names = ("boddy",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("boddy",)

  for reward_name in ["foot_clearance", "foot_swing_height", "foot_slip"]:
    cfg.rewards[reward_name].params["asset_cfg"].site_names = site_names

  # Scale foot clearance targets for small robot.
  cfg.rewards["foot_clearance"].params["target_height"] = 0.03
  cfg.rewards["foot_swing_height"].params["target_height"] = 0.03

  # Tighten velocity tracking for small robot.
  cfg.rewards["track_linear_velocity"].params["std"] = 0.2
  cfg.rewards["track_angular_velocity"].params["std"] = 0.3

  cfg.rewards["body_ang_vel"].weight = -0.05
  cfg.rewards["angular_momentum"].weight = -0.02
  cfg.rewards["air_time"].weight = 0.0

  # Penalize low base height instead of terminating.
  cfg.rewards["base_height_low"] = RewardTermCfg(
    func=mdp.root_height_below_minimum,
    weight=-2.0,
    params={"minimum_height": 0.13},
  )


  cfg.events["push_robot"].interval_range_s = (3.0, 5.0)
  cfg.events["push_robot"].params["velocity_range"] = {
    "x": (-0.1, 0.1),
    "y": (-0.1, 0.1),
    "z": (-0.05, 0.05),
    "roll": (-0.05, 0.05),
    "pitch": (-0.05, 0.05),
    "yaw": (-0.1, 0.1),
  }

  # Use motor-space observations for the parallel knee mechanism.
  cfg.observations["policy"].terms["joint_pos"] = ObservationTermCfg(
    func=joint_pos_motor
  )
  cfg.observations["critic"].terms["joint_pos"] = ObservationTermCfg(
    func=joint_pos_motor
  )
  cfg.observations["policy"].terms["joint_vel"] = ObservationTermCfg(
    func=joint_vel_motor
  )
  cfg.observations["critic"].terms["joint_vel"] = ObservationTermCfg(
    func=joint_vel_motor
  )
  cfg.scene.terrain.friction = "1.5 0.005 0.0001"
  cfg.scene.terrain.solref = "0.005 1"
  cfg.scene.terrain.solimp = "0.995 0.9995 0.001 0.5 2"
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


def leggy_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create Leggy flat terrain velocity configuration."""
  cfg = leggy_rough_env_cfg(play=play)

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
    twist_cmd.ranges.lin_vel_x = (-0.15, 0.15)
    twist_cmd.ranges.lin_vel_y = (-0.08, 0.08)
    twist_cmd.ranges.ang_vel_z = (-0.15, 0.15)

  return cfg
