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

# Base actuator gains (identified motor gains).
BASE_KP_HIPZ = 10.0
BASE_KV_HIPZ = 0.5
BASE_KP_HIP = 20.0
BASE_KV_HIP = 0.5
BASE_KP_KNEE = 20.0
BASE_KV_KNEE = 0.5
BASE_KP_ANKLE = 30.0
BASE_KV_ANKLE = 1.0

# Per-joint gain scale from identification.
# Simulation gains are: kp_sim = kp_base * scale, kv_sim = kv_base * scale.
GAIN_SCALE_BY_JOINT = {
  "hipz_left": 2.7223749315102848,
  "hipx_left": 2.597269289116872,
  "hipy_left": 3.594425225020637,
  "knee_left": 1.5065729066968843,
  "ankley_left": 0.9899055404332747,
  "anklex_left": 0.9934845924741857,
  "hipz_right": 1.2676228773957705,
  "hipx_right": 1.1571938624037024,
  "hipy_right": 1.875574499341916,
  "knee_right": 1.921946360916316,
  "ankley_right": 0.994546012136922,
  "anklex_right": 0.7898121330723573,
}

# Keep armature in actuator config at 0.0; per-joint armature is set in the MJCF.
ACTUATOR_ARMATURE = 0.0

# Effort limits (Nm) - adjust based on your motors.
HIP_EFFORT_LIMIT = 88.0
KNEE_EFFORT_LIMIT = 88.
ANKLE_EFFORT_LIMIT = 44.


LEROBOT_ACTUATOR_HIP1 = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipz_left",
  ),
  stiffness=BASE_KP_HIPZ * GAIN_SCALE_BY_JOINT["hipz_left"],
  damping=BASE_KV_HIPZ * GAIN_SCALE_BY_JOINT["hipz_left"],
  effort_limit=150.0,
  armature=ACTUATOR_ARMATURE,
)


LEROBOT_ACTUATOR_HIP1_R = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipz_right",
  ),
  stiffness=BASE_KP_HIPZ * GAIN_SCALE_BY_JOINT["hipz_right"],
  damping=BASE_KV_HIPZ * GAIN_SCALE_BY_JOINT["hipz_right"],
  effort_limit=150.0,
  armature=ACTUATOR_ARMATURE,
)


LEROBOT_ACTUATOR_HIP2 = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipx_left",
  ),
  stiffness=BASE_KP_HIP * GAIN_SCALE_BY_JOINT["hipx_left"],
  damping=BASE_KV_HIP * GAIN_SCALE_BY_JOINT["hipx_left"],
  effort_limit=88.0,
  armature=ACTUATOR_ARMATURE,
)


LEROBOT_ACTUATOR_HIP2_R = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipx_right",
  ),
  stiffness=BASE_KP_HIP * GAIN_SCALE_BY_JOINT["hipx_right"],
  damping=BASE_KV_HIP * GAIN_SCALE_BY_JOINT["hipx_right"],
  effort_limit=88.0,
  armature=ACTUATOR_ARMATURE,
)


LEROBOT_ACTUATOR_HIP3 = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipy_left",
  ),
  stiffness=BASE_KP_HIP * GAIN_SCALE_BY_JOINT["hipy_left"],
  damping=BASE_KV_HIP * GAIN_SCALE_BY_JOINT["hipy_left"],
  effort_limit=88.0,
  armature=ACTUATOR_ARMATURE,
)

LEROBOT_ACTUATOR_HIP3_R = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipy_right",
  ),
  stiffness=BASE_KP_HIP * GAIN_SCALE_BY_JOINT["hipy_right"],
  damping=BASE_KV_HIP * GAIN_SCALE_BY_JOINT["hipy_right"],
  effort_limit=88.0,
  armature=ACTUATOR_ARMATURE,
)


LEROBOT_ACTUATOR_KNEE = BuiltinPositionActuatorCfg(
  target_names_expr=("knee_left",),
  stiffness=BASE_KP_KNEE * GAIN_SCALE_BY_JOINT["knee_left"],
  damping=BASE_KV_KNEE * GAIN_SCALE_BY_JOINT["knee_left"],
  effort_limit=88.0,
  armature=ACTUATOR_ARMATURE,
)

LEROBOT_ACTUATOR_KNEE_R = BuiltinPositionActuatorCfg(
  target_names_expr=("knee_right",),
  stiffness=BASE_KP_KNEE * GAIN_SCALE_BY_JOINT["knee_right"],
  damping=BASE_KV_KNEE * GAIN_SCALE_BY_JOINT["knee_right"],
  effort_limit=88.0,
  armature=ACTUATOR_ARMATURE,
)

LEROBOT_ACTUATOR_ANKLE = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "ankley_left",
  ),
  stiffness=BASE_KP_ANKLE * GAIN_SCALE_BY_JOINT["ankley_left"],
  damping=BASE_KV_ANKLE * GAIN_SCALE_BY_JOINT["ankley_left"],
  effort_limit=44.0,
  armature=ACTUATOR_ARMATURE,
)

LEROBOT_ACTUATOR_ANKLE_R = BuiltinPositionActuatorCfg(
  target_names_expr=("ankley_right",),
  stiffness=BASE_KP_ANKLE * GAIN_SCALE_BY_JOINT["ankley_right"],
  damping=BASE_KV_ANKLE * GAIN_SCALE_BY_JOINT["ankley_right"],
  effort_limit=44.0,
  armature=ACTUATOR_ARMATURE,
)

LEROBOT_ACTUATOR_ANKLEX = BuiltinPositionActuatorCfg(
  target_names_expr=("anklex_left",),
  stiffness=BASE_KP_ANKLE * GAIN_SCALE_BY_JOINT["anklex_left"],
  damping=BASE_KV_ANKLE * GAIN_SCALE_BY_JOINT["anklex_left"],
  effort_limit=44.0,
  armature=ACTUATOR_ARMATURE,
)

LEROBOT_ACTUATOR_ANKLEX_R = BuiltinPositionActuatorCfg(
  target_names_expr=("anklex_right",),
  stiffness=BASE_KP_ANKLE * GAIN_SCALE_BY_JOINT["anklex_right"],
  damping=BASE_KV_ANKLE * GAIN_SCALE_BY_JOINT["anklex_right"],
  effort_limit=44.0,
  armature=ACTUATOR_ARMATURE,
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
    LEROBOT_ACTUATOR_HIP1_R,
    LEROBOT_ACTUATOR_HIP2,
    LEROBOT_ACTUATOR_HIP2_R,
    LEROBOT_ACTUATOR_HIP3,
    LEROBOT_ACTUATOR_HIP3_R,
    LEROBOT_ACTUATOR_KNEE,
    LEROBOT_ACTUATOR_KNEE_R,
    LEROBOT_ACTUATOR_ANKLE,
    LEROBOT_ACTUATOR_ANKLE_R,
    LEROBOT_ACTUATOR_ANKLEX,
    LEROBOT_ACTUATOR_ANKLEX_R,
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


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_lerobot_humanoid_no_arms_robot_cfg())
  viewer.launch(robot.spec.compile())
