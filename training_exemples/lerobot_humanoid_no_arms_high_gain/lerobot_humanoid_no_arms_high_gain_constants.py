"""LeRobot Humanoid (12-DOF bipedal) high-gain constants.

Model: identification from commit 3c56aee (higher friction / armature values,
       symmetric left/right, with original joint ranges ±20° hip yaw/roll).
Gains: literature-recommended values for a ~10-15 kg lightweight bipedal.
"""

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


_REPO_ROOT = _find_repo_root()

# Robot XML: 3c56aee identification (dynamics params), stored in its own model dir.
LEROBOT_HG_XML: Path = _REPO_ROOT / "models" / "bipedal_plateform_no_arms_high_gain" / "mjcf" / "robot.xml"
# Meshes are shared with the base model (geometry is unchanged).
_MESH_DIR: Path = _REPO_ROOT / "models" / "bipedal_plateform_no_arms" / "mjcf"

assert LEROBOT_HG_XML.exists(), f"MJCF file not found: {LEROBOT_HG_XML}"


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, _MESH_DIR, meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(LEROBOT_HG_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Actuator gains — literature-recommended for ~10-15 kg lightweight bipedal.
#
# References:
#   - Berkeley Humanoid (~12 kg): hip Kp≈60-80, knee Kp≈60-80
#   - Unitree H1 (47 kg, Kp=200) scaled by mass ratio ≈ 50-70
#   - Kv chosen for damping ratio ζ ≈ 0.7: Kv = 2 * 0.7 * sqrt(Kp * J_eff)
#   - Control frequency: 50 Hz (decimation=4, sim dt=5ms)
##

BASE_KP_HIPZ = 30.0
BASE_KV_HIPZ = 3.0
BASE_KP_HIPX = 40.0
BASE_KV_HIPX = 3.0
BASE_KP_HIP = 60.0   # hipy
BASE_KV_HIP = 4.0
BASE_KP_KNEE = 60.0
BASE_KV_KNEE = 4.0
BASE_KP_ANKLE = 20.0
BASE_KV_ANKLE = 1.5

# Identification from commit 3c56aee — symmetric, higher friction, physically consistent.
HIPZ_ARMATURE = 0.0226574181459
HIPZ_FRICTIONLOSS = 1.35093364028

HIPX_RIGHT_ARMATURE = 0.133269754407
HIPX_LEFT_ARMATURE = 0.133877916753
HIPX_RIGHT_FRICTIONLOSS = 1.15781879707
HIPX_LEFT_FRICTIONLOSS = 1.16683759813

HIPY_ARMATURE = 0.14077316233
HIPY_FRICTIONLOSS = 1.3122407137

KNEE_RIGHT_ARMATURE = 0.123300247512
KNEE_LEFT_ARMATURE = 0.123303314591
KNEE_RIGHT_FRICTIONLOSS = 0.91947125507
KNEE_LEFT_FRICTIONLOSS = 1.0764965485

ANKLEY_RIGHT_ARMATURE = 0.0300692570618
ANKLEY_LEFT_ARMATURE = 0.0296987477637
ANKLEY_RIGHT_FRICTIONLOSS = 0.171194932409
ANKLEY_LEFT_FRICTIONLOSS = 0.17117010829

ANKLEX_RIGHT_ARMATURE = 0.0299366151844
ANKLEX_LEFT_ARMATURE = 0.0300569538165
ANKLEX_RIGHT_FRICTIONLOSS = 0.233720916986
ANKLEX_LEFT_FRICTIONLOSS = 0.290252613159

# Effort limits (Nm).
HIP_EFFORT_LIMIT = 88.0
KNEE_EFFORT_LIMIT = 88.0
ANKLE_EFFORT_LIMIT = 44.0

##
# Actuator configs.
##

LEROBOT_HG_ACTUATOR_HIPZ_RIGHT = BuiltinPositionActuatorCfg(
  target_names_expr=("hipz_right",),
  stiffness=BASE_KP_HIPZ,
  damping=BASE_KV_HIPZ,
  effort_limit=HIP_EFFORT_LIMIT,
  armature=HIPZ_ARMATURE,
  frictionloss=HIPZ_FRICTIONLOSS,
)

LEROBOT_HG_ACTUATOR_HIPZ_LEFT = BuiltinPositionActuatorCfg(
  target_names_expr=("hipz_left",),
  stiffness=BASE_KP_HIPZ,
  damping=BASE_KV_HIPZ,
  effort_limit=HIP_EFFORT_LIMIT,
  armature=HIPZ_ARMATURE,
  frictionloss=HIPZ_FRICTIONLOSS,
)

LEROBOT_HG_ACTUATOR_HIPX_RIGHT = BuiltinPositionActuatorCfg(
  target_names_expr=("hipx_right",),
  stiffness=BASE_KP_HIPX,
  damping=BASE_KV_HIPX,
  effort_limit=HIP_EFFORT_LIMIT,
  armature=HIPX_RIGHT_ARMATURE,
  frictionloss=HIPX_RIGHT_FRICTIONLOSS,
)

LEROBOT_HG_ACTUATOR_HIPX_LEFT = BuiltinPositionActuatorCfg(
  target_names_expr=("hipx_left",),
  stiffness=BASE_KP_HIPX,
  damping=BASE_KV_HIPX,
  effort_limit=HIP_EFFORT_LIMIT,
  armature=HIPX_LEFT_ARMATURE,
  frictionloss=HIPX_LEFT_FRICTIONLOSS,
)

LEROBOT_HG_ACTUATOR_HIPY_RIGHT = BuiltinPositionActuatorCfg(
  target_names_expr=("hipy_right",),
  stiffness=BASE_KP_HIP,
  damping=BASE_KV_HIP,
  effort_limit=HIP_EFFORT_LIMIT,
  armature=HIPY_ARMATURE,
  frictionloss=HIPY_FRICTIONLOSS,
)

LEROBOT_HG_ACTUATOR_HIPY_LEFT = BuiltinPositionActuatorCfg(
  target_names_expr=("hipy_left",),
  stiffness=BASE_KP_HIP,
  damping=BASE_KV_HIP,
  effort_limit=HIP_EFFORT_LIMIT,
  armature=HIPY_ARMATURE,
  frictionloss=HIPY_FRICTIONLOSS,
)

LEROBOT_HG_ACTUATOR_KNEE_RIGHT = BuiltinPositionActuatorCfg(
  target_names_expr=("knee_right",),
  stiffness=BASE_KP_KNEE,
  damping=BASE_KV_KNEE,
  effort_limit=KNEE_EFFORT_LIMIT,
  armature=KNEE_RIGHT_ARMATURE,
  frictionloss=KNEE_RIGHT_FRICTIONLOSS,
)

LEROBOT_HG_ACTUATOR_KNEE_LEFT = BuiltinPositionActuatorCfg(
  target_names_expr=("knee_left",),
  stiffness=BASE_KP_KNEE,
  damping=BASE_KV_KNEE,
  effort_limit=KNEE_EFFORT_LIMIT,
  armature=KNEE_LEFT_ARMATURE,
  frictionloss=KNEE_LEFT_FRICTIONLOSS,
)

LEROBOT_HG_ACTUATOR_ANKLEY_RIGHT = BuiltinPositionActuatorCfg(
  target_names_expr=("ankley_right",),
  stiffness=BASE_KP_ANKLE,
  damping=BASE_KV_ANKLE,
  effort_limit=ANKLE_EFFORT_LIMIT,
  armature=ANKLEY_RIGHT_ARMATURE,
  frictionloss=ANKLEY_RIGHT_FRICTIONLOSS,
)

LEROBOT_HG_ACTUATOR_ANKLEY_LEFT = BuiltinPositionActuatorCfg(
  target_names_expr=("ankley_left",),
  stiffness=BASE_KP_ANKLE,
  damping=BASE_KV_ANKLE,
  effort_limit=ANKLE_EFFORT_LIMIT,
  armature=ANKLEY_LEFT_ARMATURE,
  frictionloss=ANKLEY_LEFT_FRICTIONLOSS,
)

LEROBOT_HG_ACTUATOR_ANKLEX_RIGHT = BuiltinPositionActuatorCfg(
  target_names_expr=("anklex_right",),
  stiffness=BASE_KP_ANKLE,
  damping=BASE_KV_ANKLE,
  effort_limit=ANKLE_EFFORT_LIMIT,
  armature=ANKLEX_RIGHT_ARMATURE,
  frictionloss=ANKLEX_RIGHT_FRICTIONLOSS,
)

LEROBOT_HG_ACTUATOR_ANKLEX_LEFT = BuiltinPositionActuatorCfg(
  target_names_expr=("anklex_left",),
  stiffness=BASE_KP_ANKLE,
  damping=BASE_KV_ANKLE,
  effort_limit=ANKLE_EFFORT_LIMIT,
  armature=ANKLEX_LEFT_ARMATURE,
  frictionloss=ANKLEX_LEFT_FRICTIONLOSS,
)

##
# Keyframe.
##

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.72),
  joint_pos={
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

FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision",),
  condim={r"^(left|right)_foot_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot_collision$": 1},
  friction={r"^(left|right)_foot_collision$": (0.6,)},
  solref={r"^(left|right)_foot_collision$": (0.005, 1.0)},
  solimp={r"^(left|right)_foot_collision$": (0.995, 0.9995, 0.001, 0.5, 2)},
)

##
# Articulation.
##

LEROBOT_HG_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    LEROBOT_HG_ACTUATOR_HIPZ_RIGHT,
    LEROBOT_HG_ACTUATOR_HIPZ_LEFT,
    LEROBOT_HG_ACTUATOR_HIPX_RIGHT,
    LEROBOT_HG_ACTUATOR_HIPX_LEFT,
    LEROBOT_HG_ACTUATOR_HIPY_RIGHT,
    LEROBOT_HG_ACTUATOR_HIPY_LEFT,
    LEROBOT_HG_ACTUATOR_KNEE_RIGHT,
    LEROBOT_HG_ACTUATOR_KNEE_LEFT,
    LEROBOT_HG_ACTUATOR_ANKLEY_RIGHT,
    LEROBOT_HG_ACTUATOR_ANKLEY_LEFT,
    LEROBOT_HG_ACTUATOR_ANKLEX_RIGHT,
    LEROBOT_HG_ACTUATOR_ANKLEX_LEFT,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_lerobot_hg_robot_cfg() -> EntityCfg:
  """Return a fresh high-gain robot config (new instance each call to avoid mutation)."""
  return EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=LEROBOT_HG_ARTICULATION,
  )


# Action scale: 0.25 * effort_limit / Kp — same formula as base config.
LEROBOT_HG_ACTION_SCALE: dict[str, float] = {}
for _a in LEROBOT_HG_ARTICULATION.actuators:
  assert isinstance(_a, BuiltinPositionActuatorCfg)
  _e = _a.effort_limit
  _s = _a.stiffness
  assert _e is not None
  for _n in _a.target_names_expr:
    LEROBOT_HG_ACTION_SCALE[_n] = 0.25 * _e / _s
