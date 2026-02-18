"""Open Duck Mini v2 (backlash) velocity environment configuration."""

import math

from .open_duck_mini_v2_constants import (
  OPEN_DUCK_MINI_V2_ACTION_SCALE,
  get_open_duck_mini_v2_robot_cfg,
)
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise
from . import mdp as open_duck_mdp


# Domain randomization toggles
ENABLE_COM_RANDOMIZATION = True
ENABLE_KP_RANDOMIZATION = True
ENABLE_KD_RANDOMIZATION = True
ENABLE_MASS_INERTIA_RANDOMIZATION = True
ENABLE_VELOCITY_PUSHES = True
ENABLE_IMU_ORIENTATION_RANDOMIZATION = True
ENABLE_BASE_ORIENTATION_RANDOMIZATION = False

# Observation configuration
USE_PROJECTED_GRAVITY = True  # If True, use projected gravity instead of raw accelerometer

# Domain randomization ranges (scaled for larger robot ~2kg, 430mm vs 0.7kg, 263mm)
COM_RANDOMIZATION_RANGE = 0.005  # ±5mm (scaled from 3mm for microduck)
MASS_INERTIA_RANDOMIZATION_RANGE = (0.95, 1.05)  # ±5%
KP_RANDOMIZATION_RANGE = (0.85, 1.15)  # ±15%
KD_RANDOMIZATION_RANGE = (0.9, 1.1)  # ±10%
VELOCITY_PUSH_INTERVAL_S = (3.0, 6.0)
VELOCITY_PUSH_RANGE = (-0.3, 0.3)  # Scaled from -0.3,0.3 for larger robot
IMU_ORIENTATION_RANDOMIZATION_ANGLE = 1.0  # ±2° IMU mounting error
BASE_ORIENTATION_MAX_PITCH_DEG = 10.0
BASE_ORIENTATION_MAX_ROLL_DEG = 5.0


def open_duck_mini_v2_velocity_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    """Create Open Duck Mini v2 velocity tracking environment configuration.

    Similar to microduck but scaled for larger robot:
    - Microduck: 0.7kg, 263mm tall, CoM ~0.08-0.11m
    - Open Duck Mini v2: 2kg, 430mm tall, CoM ~0.13-0.18m
    """

    # Joint standard deviations for pose reward
    std_walking = {
        r".*hip_yaw.*": 0.4,      # Slightly larger than microduck (0.3)
        r".*hip_roll.*": 0.3,     # Scaled from 0.2
        r".*hip_pitch.*": 0.5,    # Scaled from 0.4
        r".*knee.*": 0.5,          # Scaled from 0.4
        r".*ankle.*": 0.3,         # Scaled from 0.25
        # Head
        r".*neck.*": 0.15,
        r".*head.*": 0.15,
    }

    site_names = ["left_foot", "right_foot"]

    # Contact sensor for feet
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

    foot_frictions_geom_names = (
        "left_foot1_collision",
        "left_foot2_collision",
        "right_foot1_collision",
        "right_foot2_collision",
    )

    # Base configuration
    cfg = make_velocity_env_cfg()

    cfg.observations["critic"].terms["foot_height"].params[
        "asset_cfg"
    ].site_names = site_names

    # Robot setup
    cfg.scene.entities = {"robot": get_open_duck_mini_v2_robot_cfg()}
    cfg.scene.sensors = (feet_ground_cfg, self_collision_cfg)
    cfg.viewer.body_name = "trunk_assembly"

    # Action configuration
    joint_pos_action = cfg.actions["joint_pos"]
    assert isinstance(joint_pos_action, JointPositionActionCfg)
    joint_pos_action.scale = 1.0

    # === REWARDS ===
    # Pose reward configuration
    cfg.rewards["pose"].params["std_standing"] = std_walking
    cfg.rewards["pose"].params["std_walking"] = std_walking
    cfg.rewards["pose"].params["std_running"] = std_walking
    cfg.rewards["pose"].params["walking_threshold"] = 0.01
    cfg.rewards["pose"].weight = 3.0

    # Body-specific reward configurations
    cfg.rewards["upright"].params["asset_cfg"].body_names = ("trunk_assembly",)
    cfg.rewards["upright"].weight = 1.0

    # Foot-specific configurations
    for reward_name in ["foot_clearance", "foot_swing_height", "foot_slip"]:
        cfg.rewards[reward_name].params["asset_cfg"].site_names = site_names

    # Body-specific configurations
    cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("trunk_assembly",)

    cfg.rewards["foot_slip"].weight = -0.1
    cfg.rewards["foot_slip"].params["command_threshold"] = 0.01

    # Body dynamics rewards
    cfg.rewards["soft_landing"].weight = -1e-05

    # Air time reward - scaled for larger robot (longer swing times)
    cfg.rewards["air_time"].weight = 5.0
    cfg.rewards["air_time"].params["command_threshold"] = 0.01
    cfg.rewards["air_time"].params["threshold_min"] = 0.15  # Was 0.13
    cfg.rewards["air_time"].params["threshold_max"] = 0.4  # Was 0.32

    cfg.rewards["body_ang_vel"].weight = -0.05
    cfg.rewards["angular_momentum"].weight = -0.02

    # Velocity tracking rewards
    cfg.rewards["track_linear_velocity"].weight = 3.0
    cfg.rewards["track_linear_velocity"].params["std"] = math.sqrt(0.15)
    cfg.rewards["track_angular_velocity"].weight = 3.0
    cfg.rewards["track_angular_velocity"].params["std"] = math.sqrt(0.40)

    # Action smoothness
    cfg.rewards["action_rate_l2"].weight = -0.6

    # Foot clearance targets scaled for larger robot
    cfg.rewards["foot_clearance"].params["command_threshold"] = 0.01
    cfg.rewards["foot_clearance"].params["target_height"] = 0.02  # Scaled from 0.01 for microduck

    cfg.rewards["foot_swing_height"].params["command_threshold"] = 0.01
    cfg.rewards["foot_swing_height"].params["target_height"] = 0.02  # Scaled from 0.01

    # Neck stability
    cfg.rewards["neck_action_rate_l2"] = RewardTermCfg(
        func=open_duck_mdp.neck_action_rate_l2, weight=-0.5
    )

    # CoM height target - scaled for larger robot
    cfg.rewards["com_height_target"] = RewardTermCfg(
        func=open_duck_mdp.com_height_target,
        weight=1.2,
        params={
            "target_height_min": 0.13,  # Scaled from 0.08 (430/263 ≈ 1.63x)
            "target_height_max": 0.18,  # Scaled from 0.11
        },
    )

    # Joint torques penalty
    cfg.rewards["joint_torques_l2"] = RewardTermCfg(
        func=open_duck_mdp.joint_torques_l2, weight=-1e-3
    )

    # Events
    cfg.events["reset_action_history"] = EventTermCfg(
        func=open_duck_mdp.reset_action_history,
        mode="reset",
    )

    cfg.events["foot_friction"].params[
        "asset_cfg"
    ].geom_names = foot_frictions_geom_names
    cfg.events["reset_base"].params["pose_range"]["z"] = (0.20, 0.21)  # Scaled from 0.12-0.13

    # Velocity-based pushes for robustness training
    if ENABLE_VELOCITY_PUSHES:
        from mjlab.managers.scene_entity_config import SceneEntityCfg

        interval = (0.5, 1.0) if play else VELOCITY_PUSH_INTERVAL_S

        cfg.events["push_robot"] = EventTermCfg(
            func=mdp.push_by_setting_velocity,
            mode="interval",
            interval_range_s=interval,
            params={
                "velocity_range": {
                    "x": VELOCITY_PUSH_RANGE,
                    "y": VELOCITY_PUSH_RANGE,
                },
                "asset_cfg": SceneEntityCfg("robot"),
            },
        )

    # Domain randomization
    if ENABLE_COM_RANDOMIZATION:
        from mjlab.managers.scene_entity_config import SceneEntityCfg
        cfg.events["randomize_com"] = EventTermCfg(
            func=mdp.randomize_field,
            mode="reset",
            domain_randomization=True,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=("trunk_assembly",)),
                "operation": "add",
                "field": "body_ipos",
                "ranges": (-COM_RANDOMIZATION_RANGE, COM_RANDOMIZATION_RANGE),
            },
        )

    if ENABLE_KP_RANDOMIZATION or ENABLE_KD_RANDOMIZATION:
        from mjlab.managers.scene_entity_config import SceneEntityCfg
        kp_range = KP_RANDOMIZATION_RANGE if ENABLE_KP_RANDOMIZATION else (1.0, 1.0)
        kd_range = KD_RANDOMIZATION_RANGE if ENABLE_KD_RANDOMIZATION else (1.0, 1.0)
        cfg.events["randomize_motor_gains"] = EventTermCfg(
            func=open_duck_mdp.randomize_delayed_actuator_gains,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "operation": "scale",
                "kp_range": kp_range,
                "kd_range": kd_range,
            },
        )

    if ENABLE_MASS_INERTIA_RANDOMIZATION:
        from mjlab.managers.scene_entity_config import SceneEntityCfg
        cfg.events["randomize_mass_inertia"] = EventTermCfg(
            func=open_duck_mdp.randomize_mass_and_inertia,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=("trunk_assembly",)),
                "scale_range": MASS_INERTIA_RANDOMIZATION_RANGE,
            },
        )

    # IMU orientation randomization
    if ENABLE_IMU_ORIENTATION_RANDOMIZATION:
        from mjlab.managers.scene_entity_config import SceneEntityCfg
        cfg.events["randomize_imu_orientation"] = EventTermCfg(
            func=open_duck_mdp.randomize_imu_orientation,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "max_angle_deg": IMU_ORIENTATION_RANDOMIZATION_ANGLE,
            },
        )

    # Base orientation randomization
    if ENABLE_BASE_ORIENTATION_RANDOMIZATION:
        from mjlab.managers.scene_entity_config import SceneEntityCfg
        cfg.events["randomize_base_orientation"] = EventTermCfg(
            func=open_duck_mdp.randomize_base_orientation,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "max_pitch_deg": BASE_ORIENTATION_MAX_PITCH_DEG,
                "max_roll_deg": BASE_ORIENTATION_MAX_ROLL_DEG,
            },
        )

    # Configure joint observations to only use actuated joints (14 DoFs, not all 24 including backlash)
    actuated_joint_names = (
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
        "neck_pitch", "head_pitch", "head_yaw", "head_roll",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
    )

    from mjlab.managers.scene_entity_config import SceneEntityCfg
    actuated_joints_cfg = SceneEntityCfg("robot", joint_names=actuated_joint_names)

    # Policy observations - no base_lin_vel (privileged info for critic only)
    del cfg.observations["policy"].terms["base_lin_vel"]
    cfg.observations["policy"].terms["base_ang_vel"] = ObservationTermCfg(
        func=mdp.builtin_sensor,
        params={"sensor_name": "robot/gyro"},
    )
    cfg.observations["policy"].terms["joint_pos"].params["asset_cfg"] = actuated_joints_cfg
    cfg.observations["policy"].terms["joint_vel"].params["asset_cfg"] = actuated_joints_cfg

    # Critic observations - has base_lin_vel (privileged)
    cfg.observations["critic"].terms["base_lin_vel"] = ObservationTermCfg(
        func=mdp.builtin_sensor,
        params={"sensor_name": "robot/local_linvel"},
    )
    cfg.observations["critic"].terms["base_ang_vel"] = ObservationTermCfg(
        func=mdp.builtin_sensor,
        params={"sensor_name": "robot/gyro"},
    )
    cfg.observations["critic"].terms["joint_pos"].params["asset_cfg"] = actuated_joints_cfg
    cfg.observations["critic"].terms["joint_vel"].params["asset_cfg"] = actuated_joints_cfg

    # Determine gravity/accelerometer term name
    gravity_term_name = "projected_gravity" if USE_PROJECTED_GRAVITY else "raw_accelerometer"

    if not USE_PROJECTED_GRAVITY:
        del cfg.observations["policy"].terms["projected_gravity"]
        cfg.observations["policy"].terms["raw_accelerometer"] = ObservationTermCfg(
            func=open_duck_mdp.raw_accelerometer,
            scale=1.0,
        )

    # Add delay and noise to IMU observations
    cfg.observations["policy"].terms["base_ang_vel"].delay_min_lag = 0
    cfg.observations["policy"].terms["base_ang_vel"].delay_max_lag = 3
    cfg.observations["policy"].terms["base_ang_vel"].delay_update_period = 64
    cfg.observations["policy"].terms["base_ang_vel"].noise = Unoise(n_min=-0.024, n_max=0.024)

    if gravity_term_name in cfg.observations["policy"].terms:
        cfg.observations["policy"].terms[gravity_term_name].delay_min_lag = 0
        cfg.observations["policy"].terms[gravity_term_name].delay_max_lag = 3
        cfg.observations["policy"].terms[gravity_term_name].delay_update_period = 64
        cfg.observations["policy"].terms[gravity_term_name].noise = Unoise(n_min=-0.007, n_max=0.007)

    # Joint observation noise
    cfg.observations["policy"].terms["joint_pos"].noise = Unoise(n_min=-0.0006, n_max=0.0006)
    cfg.observations["policy"].terms["joint_vel"].noise = Unoise(n_min=-0.024, n_max=0.024)

    # Commands - slightly higher velocities for larger robot
    command: UniformVelocityCommandCfg = cfg.commands["twist"]
    command.rel_standing_envs = 0.02
    command.rel_heading_envs = 0.0
    command.ranges.lin_vel_x = (-0.3, 0.3)  # Scaled from -0.3, 0.3
    command.ranges.lin_vel_y = (-0.3, 0.3)
    command.ranges.ang_vel_z = (-1.0, 1.0)  # Keep similar angular velocity
    command.viz.z_offset = 0.8  # Scaled from 0.5 for taller robot

    # Terrain
    cfg.scene.terrain.terrain_type = "plane"
    cfg.scene.terrain.terrain_generator = None

    # Action rate curriculum
    cfg.curriculum["action_rate_weight"] = CurriculumTermCfg(
        func=mdp.reward_weight,
        params={
            "reward_name": "action_rate_l2",
            "weight_stages": [
                {"step": 0, "weight": -0.4},
                {"step": 250 * 24, "weight": -0.6},
                {"step": 500 * 24, "weight": -0.8},
                {"step": 750 * 24, "weight": -1.0},
                {"step": 1000 * 24, "weight": -1.2},
                {"step": 1250 * 24, "weight": -1.4},
                {"step": 1500 * 24, "weight": -1.6},
            ],
        },
    )

    # Velocity command ranges curriculum
    # cfg.curriculum["velocity_command_ranges"] = CurriculumTermCfg(
        # func=open_duck_mdp.velocity_command_ranges_curriculum,
        # params={
            # "command_name": "twist",
            # "velocity_stages": [
                # {"step": 0, "lin_vel_range": 0.4, "ang_vel_range": 1.5},
                # {"step": 500 * 24, "lin_vel_range": 0.5, "ang_vel_range": 1.75},
                # {"step": 1000 * 24, "lin_vel_range": 0.6, "ang_vel_range": 2.0},
            # ],
        # },
    # )

    # Disable default curriculum
    del cfg.curriculum["terrain_levels"]
    del cfg.curriculum["command_vel"]

    return cfg
