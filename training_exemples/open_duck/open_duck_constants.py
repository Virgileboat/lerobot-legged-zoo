"""Open Duck Mini v2 constants."""

from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg
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


OPEN_DUCK_MESH_DIR: Path = _find_repo_root() / "models" / "open_duck"
OPEN_DUCK_XML: Path = OPEN_DUCK_MESH_DIR / "robot.xml"

assert OPEN_DUCK_XML.exists(), f"MJCF file not found: {OPEN_DUCK_XML}"


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, OPEN_DUCK_MESH_DIR, meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(OPEN_DUCK_XML))
  spec.assets = get_assets(spec.meshdir)

  return spec


##
# Actuator config.
#
# The robot uses position-controlled actuators.
# We define conservative stiffness and damping values.
# Adjust based on your actual motor specifications.
##

# Conservative actuator parameters for small biped locomotion.
# These should be tuned based on actual motor specs.
HIP_ARMATURE = 0.01  # Reflected inertia for hip joints.
KNEE_ARMATURE = 0.008  # Reflected inertia for knee joints.
ANKLE_ARMATURE = 0.006  # Reflected inertia for ankle joints.

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

# Stiffness and damping derived from armature.
HIP_STIFFNESS = HIP_ARMATURE * NATURAL_FREQ**2
KNEE_STIFFNESS = KNEE_ARMATURE * NATURAL_FREQ**2
ANKLE_STIFFNESS = ANKLE_ARMATURE * NATURAL_FREQ**2

HIP_DAMPING = 2.0 * DAMPING_RATIO * HIP_ARMATURE * NATURAL_FREQ
KNEE_DAMPING = 2.0 * DAMPING_RATIO * KNEE_ARMATURE * NATURAL_FREQ
ANKLE_DAMPING = 2.0 * DAMPING_RATIO * ANKLE_ARMATURE * NATURAL_FREQ

# Effort limits (Nm) - adjust based on your motors.
HIP_EFFORT_LIMIT = 5.0
KNEE_EFFORT_LIMIT = 5.0
ANKLE_EFFORT_LIMIT = 5.0
UPPER_EFFORT_LIMIT = 2.0

OPEN_DUCK_ACTUATOR_HIP = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
  ),
  stiffness=HIP_STIFFNESS,
  damping=HIP_DAMPING,
  effort_limit=HIP_EFFORT_LIMIT,
  armature=HIP_ARMATURE,
)

OPEN_DUCK_ACTUATOR_KNEE = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "left_knee",
    "right_knee",
  ),
  stiffness=KNEE_STIFFNESS,
  damping=KNEE_DAMPING,
  effort_limit=KNEE_EFFORT_LIMIT,
  armature=KNEE_ARMATURE,
)

OPEN_DUCK_ACTUATOR_ANKLE = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "left_ankle",
    "right_ankle",
  ),
  stiffness=ANKLE_STIFFNESS,
  damping=ANKLE_DAMPING,
  effort_limit=ANKLE_EFFORT_LIMIT,
  armature=ANKLE_ARMATURE,
)

OPEN_DUCK_ACTUATOR_UPPER = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "left_antenna",
    "right_antenna",
  ),
  stiffness=ANKLE_STIFFNESS,
  damping=ANKLE_DAMPING,
  effort_limit=UPPER_EFFORT_LIMIT,
  armature=ANKLE_ARMATURE,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.32),
  joint_pos={
    # Standing pose - all joints at zero.
    ".*": 0.0,
    "left_hip_pitch": -0.4,
    "right_hip_pitch": 0.4,
    "left_knee": 0.8,
    "right_knee": 0.8,
    "left_ankle": -0.4,
    "right_ankle": -0.4,
  },
  joint_vel={".*": 0.0},
)

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.20),
  joint_pos={
    # More bent stance for better initial stability.
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
# Collision config.
##

# Enable foot collisions with appropriate friction.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(
    "left_foot1_collision",
    "left_foot2_collision",
    "right_foot1_collision",
    "right_foot2_collision",
  ),
  # contype=0 disables contacts entirely; keep it enabled for ground contact.
  contype=1,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
)

# Full collision including self-collisions.
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
    "left_foot1_collision": (0.6,),
    "left_foot2_collision": (0.6,),
    "right_foot1_collision": (0.6,),
    "right_foot2_collision": (0.6,),
  },
)

##
# Final config.
##

OPEN_DUCK_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    OPEN_DUCK_ACTUATOR_HIP,
    OPEN_DUCK_ACTUATOR_KNEE,
    OPEN_DUCK_ACTUATOR_ANKLE,
    OPEN_DUCK_ACTUATOR_UPPER,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_open_duck_robot_cfg() -> EntityCfg:
  """Get a fresh Open Duck Mini v2 robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=OPEN_DUCK_ARTICULATION,
  )


# Action scale: scales normalized actions to joint position offsets.
OPEN_DUCK_ACTION_SCALE: dict[str, float] = {}
for a in OPEN_DUCK_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    OPEN_DUCK_ACTION_SCALE[n] = 0.25 * e / s


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_open_duck_robot_cfg())
  viewer.launch(robot.spec.compile())
