"""Open Duck Mini v2 (backlash) constants."""

from pathlib import Path

import mujoco

from mjlab.actuator import DelayedActuatorCfg, XmlPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

from .actuator import make_bam_m6_actuator_cfg

##
# MJCF and assets.
##

def _find_repo_root() -> Path:
  here = Path(__file__).resolve()
  for parent in here.parents:
    if (parent / "models").is_dir():
      return parent
  raise FileNotFoundError("Could not locate repo root containing 'models' directory.")


OPEN_DUCK_MINI_V2_MESH_DIR: Path = _find_repo_root() / "models" / "open_duck_mini_v2"
OPEN_DUCK_MINI_V2_XML: Path = OPEN_DUCK_MINI_V2_MESH_DIR / "open_duck_mini_v2.xml"

assert OPEN_DUCK_MINI_V2_XML.exists(), f"MJCF file not found: {OPEN_DUCK_V2_MINI_XML}"


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, OPEN_DUCK_MINI_V2_MESH_DIR / meshdir, meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(OPEN_DUCK_MINI_V2_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Initial pose.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.22),
  joint_pos={"*": 0.0},
  joint_vel={".*": 0.0},
)

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.22),
  joint_pos={
    "left_hip_pitch": -0.4,
    "right_hip_pitch": 0.4,
    "left_knee": 0.8,
    "right_knee": 0.8,
    "left_ankle": -0.4,
    "right_ankle": -0.4,
    ".*": 0.0,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config (mesh-based).
##

FULL_COLLISION = CollisionCfg(
  geom_names_expr=(
    "left_foot_collision",
    "right_foot_collision",
  ),
  condim={
    "left_foot_collision": 3,
    "right_foot_collision": 3,
  },
  priority={
    "left_foot_collision": 1,
    "right_foot_collision": 1,
  },
  friction={
    "left_foot_collision": (1.2, 0.005, 0.0001),
    "right_foot_collision": (1.2, 0.005, 0.0001),
  },
  solref={
    "left_foot_collision": (0.01, 1.0),
    "right_foot_collision": (0.01, 1.0),
  },
  solimp={
    "left_foot_collision": (0.99, 0.999, 0.001, 0.5, 2),
    "right_foot_collision": (0.99, 0.999, 0.001, 0.5, 2),
  },
)

##
# Actuators (XML-defined, exclude backlash joints).
##

ACTUATED_JOINTS = (
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

# XML position actuator (MuJoCo built-in PD + frictionloss) — original path.
OPEN_DUCK_MINI_V2_ACTUATORS = DelayedActuatorCfg(
  delay_min_lag=0,
  delay_max_lag=3,
  base_cfg=XmlPositionActuatorCfg(joint_names_expr=ACTUATED_JOINTS),
)

# BAM M6 actuator (full voltage control + load-dependent friction).
# NOTE: we don't yet have M6 identification data for the STS3215 actuators
# used on the Open Duck Mini v2 — the default params are XL330 placeholders.
# Swap in a real STS3215 JSON via `json_path=` once measurements are done.
OPEN_DUCK_MINI_V2_M6_ACTUATORS = DelayedActuatorCfg(
  delay_min_lag=0,
  delay_max_lag=3,
  base_cfg=make_bam_m6_actuator_cfg(joint_names_expr=ACTUATED_JOINTS),
)


def get_open_duck_mini_v2_robot_cfg(use_m6: bool = False) -> EntityCfg:
  """Get a fresh Open Duck Mini v2 (backlash) robot configuration instance.

  Args:
    use_m6: If True, use the BAM M6 actuator (placeholder XL330 params until
      STS3215 identification is available). Otherwise use the XML position
      actuator wrapped in a DelayedActuator.
  """
  actuators = OPEN_DUCK_MINI_V2_M6_ACTUATORS if use_m6 else OPEN_DUCK_MINI_V2_ACTUATORS
  return EntityCfg(
    spec_fn=get_spec,
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    articulation=EntityArticulationInfoCfg(
      actuators=(actuators,),
      soft_joint_pos_limit_factor=0.9,
    ),
  )


# Action scale: fixed for now (XML actuators already define kp/forcerange).
OPEN_DUCK_MINI_V2_ACTION_SCALE = 1.0
