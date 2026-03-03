"""LeRobot Humanoid (12-DOF bipedal) constants."""

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


LEROBOT_HUMANOID_NO_ARMS_MESH_DIR: Path = _find_repo_root() / "models" / "bipedal_plateform_no_arms" / "mjcf"
LEROBOT_HUMANOID_NO_ARMS_XML: Path = LEROBOT_HUMANOID_NO_ARMS_MESH_DIR / "robot.xml"

assert LEROBOT_HUMANOID_NO_ARMS_XML.exists(), f"MJCF file not found: {LEROBOT_HUMANOID_NO_ARMS_XML}"


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, LEROBOT_HUMANOID_NO_ARMS_MESH_DIR, meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(LEROBOT_HUMANOID_NO_ARMS_XML))
  spec.assets = get_assets(spec.meshdir)

  return spec


##
# Actuator config.
#
# The robot uses position-controlled actuators.
# We define conservative stiffness and damping values.
# Adjust based on your actual motor specifications.
##

# Identified actuator parameters.
HIPZ_ARMATURE = 0.03630317163452016
HIPX_ARMATURE = 0.04682185962389506
HIPY_ARMATURE = 0.04805047840123755
KNEE_ARMATURE = 0.04805047840123755
ANKLE_ARMATURE = 0.01  # Reflected inertia for ankle joints.

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

# Identified gains.
HIPZ_STIFFNESS = 98.92525911421382
HIPX_STIFFNESS = 305.3819028946636
HIPY_STIFFNESS = 119.93582922139161
KNEE_STIFFNESS = 119.93582922139161
ANKLE_STIFFNESS = 60.0

HIPZ_DAMPING = 2.430158270946069
HIPX_DAMPING = 3.5444373834683174
HIPY_DAMPING = 4.140316075357987
KNEE_DAMPING = 4.140316075357987
ANKLE_DAMPING = 2.

# Identified torque limits (Nm).
HIPZ_EFFORT_LIMIT = 131.86793225923452
HIPX_EFFORT_LIMIT = 100.0
HIPY_EFFORT_LIMIT = 95.72569340785736
KNEE_EFFORT_LIMIT = 95.72569340785736
ANKLE_EFFORT_LIMIT = 44.


LEROBOT_ACTUATOR_HIP1 = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipz_.*",
  ),
  stiffness=HIPZ_STIFFNESS,
  damping=HIPZ_DAMPING,
  effort_limit=HIPZ_EFFORT_LIMIT,
  armature=HIPZ_ARMATURE,
)


LEROBOT_ACTUATOR_HIP2 = BuiltinPositionActuatorCfg(
  target_names_expr=(
        "hipx_.*",
  ),
  stiffness=HIPX_STIFFNESS,
  damping=HIPX_DAMPING,
  effort_limit=HIPX_EFFORT_LIMIT,
  armature=HIPX_ARMATURE,
)


LEROBOT_ACTUATOR_HIP3 = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipy_.*",
  ),
  stiffness=HIPY_STIFFNESS,
  damping=HIPY_DAMPING,
  effort_limit=HIPY_EFFORT_LIMIT,
  armature=HIPY_ARMATURE,
)
LEROBOT_ACTUATOR_KNEE = BuiltinPositionActuatorCfg(
  target_names_expr=("knee_.*",),
  stiffness=KNEE_STIFFNESS,
  damping=KNEE_DAMPING,
  effort_limit=KNEE_EFFORT_LIMIT,
  armature=KNEE_ARMATURE,
)

LEROBOT_ACTUATOR_ANKLE = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "ankley_.*",
    "anklex_.*",
  ),
  stiffness=ANKLE_STIFFNESS,
  damping=ANKLE_DAMPING,
  effort_limit=44.0,
  armature=ANKLE_ARMATURE,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.78),
  joint_pos={
    # Slight crouch reference pose used as the robot default posture.
    # This becomes the target posture for the variable_posture reward
    # (it reads asset.data.default_joint_pos).
    "hipz_right": 0.0,
    "hipx_right": 0.0,
    "hipy_right": 0.0,
    "knee_right": 0.0,
    "ankley_right": 0.0,
    "anklex_right": 0.0,
    "hipz_left": 0.0,
    "hipx_left": 0.0,
    "hipy_left": -0.0,
    "knee_left": 0.0,
    "ankley_left": -0.0,
    "anklex_left": 0.0,
  },
  joint_vel={".*": 0.0},
)

# KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
#   pos=(0, 0, 0.77),
#   joint_pos={
#     # Slight crouch reference pose used as the robot default posture.
#     # This becomes the target posture for the variable_posture reward
#     # (it reads asset.data.default_joint_pos).
#     "hipz_right": 0.0,
#     "hipx_right": 0.0,
#     "hipy_right": 0.35,
#     "knee_right": 0.70,
#     "ankley_right": 0.35,
#     "anklex_right": 0.0,
#     "hipz_left": 0.0,
#     "hipx_left": 0.0,
#     "hipy_left": -0.35,
#     "knee_left": 0.70,
#     "ankley_left": -0.35,
#     "anklex_left": 0.0,
#   },
#   joint_vel={".*": 0.0},
# )


KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.77),
  joint_pos={
    # Slight crouch reference pose used as the robot default posture.
    # This becomes the target posture for the variable_posture reward
    # (it reads asset.data.default_joint_pos).
    "hipz_right": 0.0,
    "hipx_right": 0.0,
    "hipy_right": 0.0,
    "knee_right": 0.0,
    "ankley_right": 0.0,
    "anklex_right": 0.0,
    "hipz_left": 0.0,
    "hipx_left": 0.0,
    "hipy_left": -0.0,
    "knee_left": 0.0,
    "ankley_left": -0.0,
    "anklex_left": 0.0,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

# Enable foot collisions with appropriate friction.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(r"^(left|right)_foot[1-4]_collision$",),
  # contype=0 disables contacts entirely; keep it enabled for ground contact.
  contype=1,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
)

# Full collision including self-collisions.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision",),
  condim={r"^(left|right)_foot[1-4]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1-4]_collision$": 1},
  friction={r"^(left|right)_foot[1-4]_collision$": (0.6,)},
)
##
# Final config.
##

LEROBOT_HUMANOID_NO_ARMS_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    LEROBOT_ACTUATOR_HIP1,
    LEROBOT_ACTUATOR_HIP2,
    LEROBOT_ACTUATOR_HIP3,
    LEROBOT_ACTUATOR_KNEE,
    LEROBOT_ACTUATOR_ANKLE,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_lerobot_humanoid_no_arms_robot_cfg() -> EntityCfg:
  """Get a fresh LeRobot Humanoid robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=LEROBOT_HUMANOID_NO_ARMS_ARTICULATION,
  )


# Action scale: scales normalized actions to joint position offsets.
LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE: dict[str, float] = {}
for a in LEROBOT_HUMANOID_NO_ARMS_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE[n] = 0.25 * e / s

# Keep hipx action amplitude bounded for stability.
LEROBOT_HUMANOID_NO_ARMS_ACTION_SCALE[r"hipx_.*"] = 0.3


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_lerobot_humanoid_no_arms_robot_cfg())
  viewer.launch(robot.spec.compile())
