"""Open Duck v2 (backlash) velocity environment configurations."""

from .open_duck_v2_constants import (
  OPEN_DUCK_V2_ACTION_SCALE,
  get_open_duck_v2_robot_cfg,
)
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg


def open_duck_v2_rough_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create Open Duck v2 (backlash) rough terrain velocity configuration."""
  cfg = make_velocity_env_cfg()

  cfg.sim.mujoco.ccd_iterations = 500
  cfg.sim.contact_sensor_maxmatch = 500
  cfg.sim.nconmax = 45

  cfg.scene.entities = {"robot": get_open_duck_v2_robot_cfg()}

  site_names = ("left_foot", "right_foot")
  geom_names = (
    "left_foot1_collision",
    "left_foot2_collision",
    "right_foot1_collision",
    "right_foot2_collision",
  )

  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(
      mode="subtree",
      pattern=r"^(foot_assembly|foot_assembly_2)$",
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
    primary=ContactMatch(mode="subtree", pattern="trunk_assembly", entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern="trunk_assembly", entity="robot"),
    fields=("found",),
    reduce="none",
    num_slots=1,
  )
  cfg.scene.sensors = (feet_ground_cfg, self_collision_cfg)

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

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = OPEN_DUCK_V2_ACTION_SCALE
  joint_pos_action.actuator_names = (
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
  )

  cfg.viewer.body_name = "trunk_assembly"

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.viz.z_offset = 0.4  # Adjust based on robot height.
  twist_cmd.ranges.lin_vel_x = (-0.3, 0.3)
  twist_cmd.ranges.lin_vel_y = (-0.1, 0.1)
  twist_cmd.ranges.ang_vel_z = (-0.5, 0.5)

  cfg.observations["critic"].terms["foot_height"].params[
    "asset_cfg"
  ].site_names = site_names

  cfg.events["foot_friction"].params["asset_cfg"].geom_names = geom_names
  cfg.events["base_com"].params["asset_cfg"].body_names = ("trunk_assembly",)

  # Pose reward std values for actuated joints only (exclude backlash joints).
  cfg.rewards["pose"].params["asset_cfg"].joint_names = (
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
  )
  cfg.rewards["pose"].params["std_standing"] = {".*": 0.05}
  cfg.rewards["pose"].params["std_walking"] = {
    "left_hip_yaw": 0.3,
    "left_hip_roll": 0.3,
    "left_hip_pitch": 0.3,
    "right_hip_yaw": 0.3,
    "right_hip_roll": 0.3,
    "right_hip_pitch": 0.3,
    "left_knee": 0.35,
    "right_knee": 0.35,
    "left_ankle": 0.25,
    "right_ankle": 0.25,
    "neck_pitch": 0.15,
    "head_pitch": 0.15,
    "head_yaw": 0.15,
    "head_roll": 0.15,
  }
  
  cfg.rewards["pose"].params["std_running"] = {
    "left_hip_yaw": 0.5,
    "left_hip_roll": 0.5,
    "left_hip_pitch": 0.5,
    "right_hip_yaw": 0.5,
    "right_hip_roll": 0.5,
    "right_hip_pitch": 0.5,
    "left_knee": 0.6,
    "right_knee": 0.6,
    "left_ankle": 0.35,
    "right_ankle": 0.35,
    "neck_pitch": 0.25,
    "head_pitch": 0.25,
    "head_yaw": 0.25,
    "head_roll": 0.25,
  }

  cfg.rewards["upright"].params["asset_cfg"].body_names = ("trunk_assembly",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("trunk_assembly",)

  for reward_name in ["foot_clearance", "foot_swing_height", "foot_slip"]:
    cfg.rewards[reward_name].params["asset_cfg"].site_names = site_names

  # Scale foot clearance targets for ~0.5m robot.
  cfg.rewards["foot_clearance"].params["target_height"] = 0.05
  cfg.rewards["foot_swing_height"].params["target_height"] = 0.05

  # Tighten velocity tracking for small robot.
  cfg.rewards["track_linear_velocity"].params["std"] = 0.15
  cfg.rewards["track_angular_velocity"].params["std"] = 0.35
  cfg.rewards["track_linear_velocity"].weight = 4.0
  cfg.rewards["track_angular_velocity"].weight = 3.0

  # Reduce posture dominance to encourage velocity tracking.
  cfg.rewards["upright"].weight = 0.5
  cfg.rewards["pose"].weight = 0.5

  cfg.rewards["body_ang_vel"].weight = -0.05
  cfg.rewards["angular_momentum"].weight = -0.02
  cfg.rewards["air_time"].weight = 0.0

  cfg.events["push_robot"].interval_range_s = (2.0, 4.0)
  cfg.events["push_robot"].params["velocity_range"] = {
    "x": (-0.2, 0.2),
    "y": (-0.2, 0.2),
    "z": (-0.1, 0.1),
    "roll": (-0.1, 0.1),
    "pitch": (-0.1, 0.1),
    "yaw": (-0.2, 0.2),
  }

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

  # Override IMU sensor names from the XML.
  cfg.observations["policy"].terms["base_lin_vel"] = ObservationTermCfg(
    func=mdp.builtin_sensor,
    params={"sensor_name": "robot/local_linvel"},
  )
  cfg.observations["policy"].terms["base_ang_vel"] = ObservationTermCfg(
    func=mdp.builtin_sensor,
    params={"sensor_name": "robot/gyro"},
  )
  cfg.observations["critic"].terms["base_lin_vel"] = ObservationTermCfg(
    func=mdp.builtin_sensor,
    params={"sensor_name": "robot/local_linvel"},
  )
  cfg.observations["critic"].terms["base_ang_vel"] = ObservationTermCfg(
    func=mdp.builtin_sensor,
    params={"sensor_name": "robot/gyro"},
  )

  return cfg


def open_duck_v2_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create Open Duck v2 (backlash) flat terrain velocity configuration."""
  cfg = open_duck_v2_rough_env_cfg(play=play)

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
    twist_cmd.ranges.lin_vel_x = (-0.3, 0.3)
    twist_cmd.ranges.lin_vel_y = (-0.1, 0.1)
    twist_cmd.ranges.ang_vel_z = (-0.5, 0.5)

  return cfg
