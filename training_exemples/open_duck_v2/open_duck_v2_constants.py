"""Open Duck v2 (backlash) constants."""

from pathlib import Path

import mujoco

from mjlab.actuator import XmlPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

def _find_repo_root() -> Path:
  here = Path(__file__).resolve()
  for parent in here.parents:
    if (parent / "models").is_dir():
      return parent
  raise FileNotFoundError("Could not locate repo root containing 'models' directory.")


OPEN_DUCK_V2_MESH_DIR: Path = _find_repo_root() / "models" / "open_duck_v2"
OPEN_DUCK_V2_XML: Path = OPEN_DUCK_V2_MESH_DIR / "open_duck_v2_backlash.xml"

assert OPEN_DUCK_V2_XML.exists(), f"MJCF file not found: {OPEN_DUCK_V2_XML}"


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, OPEN_DUCK_V2_MESH_DIR, meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(OPEN_DUCK_V2_XML))
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
    "left_foot1_collision",
    "left_foot2_collision",
    "right_foot1_collision",
    "right_foot2_collision",
  ),
  condim={
    "left_foot1_collision": 3,
    "left_foot2_collision": 3,
    "right_foot1_collision": 3,
    "right_foot2_collision": 3,
  },
  priority={
    "left_foot1_collision": 1,
    "left_foot2_collision": 1,
    "right_foot1_collision": 1,
    "right_foot2_collision": 1,
  },
  friction={
    "left_foot1_collision": (1.2, 0.005, 0.0001),
    "left_foot2_collision": (1.2, 0.005, 0.0001),
    "right_foot1_collision": (1.2, 0.005, 0.0001),
    "right_foot2_collision": (1.2, 0.005, 0.0001),
  },
  solref={
    "left_foot1_collision": (0.01, 1.0),
    "left_foot2_collision": (0.01, 1.0),
    "right_foot1_collision": (0.01, 1.0),
    "right_foot2_collision": (0.01, 1.0),
  },
  solimp={
    "left_foot1_collision": (0.99, 0.999, 0.001, 0.5, 2),
    "left_foot2_collision": (0.99, 0.999, 0.001, 0.5, 2),
    "right_foot1_collision": (0.99, 0.999, 0.001, 0.5, 2),
    "right_foot2_collision": (0.99, 0.999, 0.001, 0.5, 2),
  },
)

##
# Actuators (XML-defined, exclude backlash joints).
##

OPEN_DUCK_V2_ACTUATORS = XmlPositionActuatorCfg(
  target_names_expr=(
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
  ),
)

OPEN_DUCK_V2_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(OPEN_DUCK_V2_ACTUATORS,),
  soft_joint_pos_limit_factor=0.9,
)


def get_open_duck_v2_robot_cfg() -> EntityCfg:
  """Get a fresh Open Duck v2 (backlash) robot configuration instance."""
  return EntityCfg(
    spec_fn=get_spec,
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    articulation=OPEN_DUCK_V2_ARTICULATION,
  )


# Action scale: fixed for now (XML actuators already define kp/forcerange).
OPEN_DUCK_V2_ACTION_SCALE = 0.5
