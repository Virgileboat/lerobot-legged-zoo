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

# Base actuator gains (no additional scaling).
BASE_KP_HIPZ = 80.0
BASE_KV_HIPZ = 4.0
BASE_KP_HIPX = 80.0
BASE_KV_HIPX = 4.0
BASE_KP_HIP = 100.0
BASE_KV_HIP = 5.0
BASE_KP_KNEE = 100.0
BASE_KV_KNEE = 5.0
BASE_KP_ANKLE = 30.0
BASE_KV_ANKLE = 1.0

# Joint armature/frictionloss applied through actuator config.
# Keep these aligned with the robot MJCF values.
HIPZ_ARMATURE = 0.113945079047333
HIPX_RIGHT_ARMATURE = 0.190069428245805
HIPX_LEFT_ARMATURE = 0.190069428245805
HIPY_ARMATURE = 0.0490681384046139
KNEE_RIGHT_ARMATURE = 0.306070112952606
KNEE_LEFT_ARMATURE = 0.306070112952606
ANKLEY_RIGHT_ARMATURE = 0.0336582365344168
ANKLEY_LEFT_ARMATURE = 0.0336582365344168
ANKLEX_RIGHT_ARMATURE = 0.0279651657663248
ANKLEX_LEFT_ARMATURE = 0.0279651657663248

HIPZ_FRICTIONLOSS = 0.807973499030501
HIPX_RIGHT_FRICTIONLOSS = 0.758840917979376
HIPX_LEFT_FRICTIONLOSS = 0.758840917979376
HIPY_FRICTIONLOSS = 0.286109814959772
KNEE_RIGHT_FRICTIONLOSS = 2.48145597014291
KNEE_LEFT_FRICTIONLOSS = 2.48145597014291
ANKLEY_RIGHT_FRICTIONLOSS = 0.271871566258512
ANKLEY_LEFT_FRICTIONLOSS = 0.271871566258512
ANKLEX_RIGHT_FRICTIONLOSS = 0.426109703383077
ANKLEX_LEFT_FRICTIONLOSS = 0.426109703383077

# Effort limits (Nm) - adjust based on your motors.
HIP_EFFORT_LIMIT = 88.0
KNEE_EFFORT_LIMIT = 88.
ANKLE_EFFORT_LIMIT = 44.


LEROBOT_ACTUATOR_HIPZ = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipz_.*",
  ),
  stiffness=BASE_KP_HIPZ,
  damping=BASE_KV_HIPZ,
  effort_limit=150.0,
  armature=HIPZ_ARMATURE,
  frictionloss=HIPZ_FRICTIONLOSS,
)


LEROBOT_ACTUATOR_HIPX_RIGHT = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipx_right",
  ),
  stiffness=BASE_KP_HIPX,
  damping=BASE_KV_HIPX,
  effort_limit=88.0,
  armature=HIPX_RIGHT_ARMATURE,
  frictionloss=HIPX_RIGHT_FRICTIONLOSS,
)


LEROBOT_ACTUATOR_HIPX_LEFT = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipx_left",
  ),
  stiffness=BASE_KP_HIPX,
  damping=BASE_KV_HIPX,
  effort_limit=88.0,
  armature=HIPX_LEFT_ARMATURE,
  frictionloss=HIPX_LEFT_FRICTIONLOSS,
)


LEROBOT_ACTUATOR_HIPY = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "hipy_.*",
  ),
  stiffness=BASE_KP_HIP,
  damping=BASE_KV_HIP,
  effort_limit=88.0,
  armature=HIPY_ARMATURE,
  frictionloss=HIPY_FRICTIONLOSS,
)


LEROBOT_ACTUATOR_KNEE_RIGHT = BuiltinPositionActuatorCfg(
  target_names_expr=("knee_right",),
  stiffness=BASE_KP_KNEE,
  damping=BASE_KV_KNEE,
  effort_limit=88.0,
  armature=KNEE_RIGHT_ARMATURE,
  frictionloss=KNEE_RIGHT_FRICTIONLOSS,
)

LEROBOT_ACTUATOR_KNEE_LEFT = BuiltinPositionActuatorCfg(
  target_names_expr=("knee_left",),
  stiffness=BASE_KP_KNEE,
  damping=BASE_KV_KNEE,
  effort_limit=88.0,
  armature=KNEE_LEFT_ARMATURE,
  frictionloss=KNEE_LEFT_FRICTIONLOSS,
)

LEROBOT_ACTUATOR_ANKLEY_RIGHT = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "ankley_right",
  ),
  stiffness=BASE_KP_ANKLE,
  damping=BASE_KV_ANKLE,
  effort_limit=44.0,
  armature=ANKLEY_RIGHT_ARMATURE,
  frictionloss=ANKLEY_RIGHT_FRICTIONLOSS,
)

LEROBOT_ACTUATOR_ANKLEY_LEFT = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "ankley_left",
  ),
  stiffness=BASE_KP_ANKLE,
  damping=BASE_KV_ANKLE,
  effort_limit=44.0,
  armature=ANKLEY_LEFT_ARMATURE,
  frictionloss=ANKLEY_LEFT_FRICTIONLOSS,
)

LEROBOT_ACTUATOR_ANKLEX_RIGHT = BuiltinPositionActuatorCfg(
  target_names_expr=("anklex_right",),
  stiffness=BASE_KP_ANKLE,
  damping=BASE_KV_ANKLE,
  effort_limit=44.0,
  armature=ANKLEX_RIGHT_ARMATURE,
  frictionloss=ANKLEX_RIGHT_FRICTIONLOSS,
)

LEROBOT_ACTUATOR_ANKLEX_LEFT = BuiltinPositionActuatorCfg(
  target_names_expr=("anklex_left",),
  stiffness=BASE_KP_ANKLE,
  damping=BASE_KV_ANKLE,
  effort_limit=44.0,
  armature=ANKLEX_LEFT_ARMATURE,
  frictionloss=ANKLEX_LEFT_FRICTIONLOSS,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.72),
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
  pos=(0, 0, 0.72),
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
  geom_names_expr=(r"^(left|right)_foot_collision$",),
  # contype=0 disables contacts entirely; keep it enabled for ground contact.
  contype=1,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
  # Stiffer contact settings to reduce penetration.
  solref=(0.005, 1.0),
  solimp=(0.995, 0.9995, 0.001, 0.5, 2),
)

# Full collision including self-collisions.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision",),
  condim={r"^(left|right)_foot_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot_collision$": 1},
  friction={r"^(left|right)_foot_collision$": (0.6,)},
  # Foot-specific stiff contact settings to reduce penetration.
  solref={r"^(left|right)_foot_collision$": (0.005, 1.0)},
  solimp={r"^(left|right)_foot_collision$": (0.995, 0.9995, 0.001, 0.5, 2)},
)
##
# Final config.
##

LEROBOT_HUMANOID_NO_ARMS_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    LEROBOT_ACTUATOR_HIPZ,
    LEROBOT_ACTUATOR_HIPX_RIGHT,
    LEROBOT_ACTUATOR_HIPX_LEFT,
    LEROBOT_ACTUATOR_HIPY,
    LEROBOT_ACTUATOR_KNEE_RIGHT,
    LEROBOT_ACTUATOR_KNEE_LEFT,
    LEROBOT_ACTUATOR_ANKLEY_RIGHT,
    LEROBOT_ACTUATOR_ANKLEY_LEFT,
    LEROBOT_ACTUATOR_ANKLEX_RIGHT,
    LEROBOT_ACTUATOR_ANKLEX_LEFT,
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


# Fixed command transport delay per joint (in simulation steps).
# Kept explicit per joint so left/right and each DOF can be tuned independently.
LEROBOT_HUMANOID_NO_ARMS_ACTION_DELAY_STEPS: dict[str, int] = {
  "hipz_right": 4,
  "hipx_right": 4,
  "hipy_right": 4,
  "knee_right": 4,
  "ankley_right": 4,
  "anklex_right": 4,
  "hipz_left": 2,
  "hipx_left": 2,
  "hipy_left": 2,
  "knee_left": 2,
  "ankley_left": 2,
  "anklex_left": 2,
}


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_lerobot_humanoid_no_arms_robot_cfg())
  viewer.launch(robot.spec.compile())
