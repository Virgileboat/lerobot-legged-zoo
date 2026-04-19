"""Load BAM M6 parameters from JSON and create actuator configs."""

from __future__ import annotations

import json
from pathlib import Path

from .bam_actuator import BamM6ActuatorCfg


# Placeholder M6 params. We don't yet have M6 identification data for the
# STS3215 actuators used on the Open Duck Mini v2, so these XL330 numbers are
# used as a stand-in until we run a proper BAM identification on STS3215.
DEFAULT_XL330_M6 = {
    "kt": 0.24702827088535634,
    "R": 2.436537942885361,
    "armature": 0.002231042413951293,
    "friction_base": 0.007805203011273793,
    "friction_stribeck": 0.01299013941785831,
    "load_friction_motor": 0.17679071496643342,
    "load_friction_external": 0.33284617369197755,
    "load_friction_motor_stribeck": 0.04834054555210131,
    "load_friction_external_stribeck": 0.03230746746292114,
    "load_friction_motor_quad": 0.004778286363709164,
    "load_friction_external_quad": 0.004335373885291851,
    "dtheta_stribeck": 0.10838180452009236,
    "alpha": 2.1089115156897034,
    "friction_viscous": 0.01674718702359746,
}


def load_bam_m6_params(json_path: str | Path) -> dict:
    """Load BAM M6 parameters from a JSON file."""
    with open(json_path) as f:
        params = json.load(f)
    assert params.get("model") == "m6", f"Expected M6 model, got {params.get('model')}"
    return params


def make_bam_m6_actuator_cfg(
    joint_names_expr: tuple[str, ...] = (r".*",),
    params: dict | None = None,
    json_path: str | Path | None = None,
    vin: float = 7.4,
    kp_fw: float = 200.0,
) -> BamM6ActuatorCfg:
    """Create a BamM6ActuatorCfg from BAM parameters.

    Args:
        joint_names_expr: Joint name patterns to control.
        params: Dict of M6 params. If None, uses DEFAULT_XL330_M6.
        json_path: Path to BAM M6 JSON. Overrides params if provided.
        vin: Supply voltage.
        kp_fw: Firmware position P gain.
    """
    if json_path is not None:
        p = load_bam_m6_params(json_path)
    elif params is not None:
        p = params
    else:
        p = DEFAULT_XL330_M6

    return BamM6ActuatorCfg(
        joint_names_expr=joint_names_expr,
        armature=p["armature"],
        kt=p["kt"],
        R=p["R"],
        vin=vin,
        kp_fw=kp_fw,
        friction_base=p["friction_base"],
        friction_stribeck=p["friction_stribeck"],
        dtheta_stribeck=p["dtheta_stribeck"],
        alpha=p["alpha"],
        friction_viscous=p["friction_viscous"],
        load_friction_motor=p["load_friction_motor"],
        load_friction_external=p["load_friction_external"],
        load_friction_motor_stribeck=p["load_friction_motor_stribeck"],
        load_friction_external_stribeck=p["load_friction_external_stribeck"],
        load_friction_motor_quad=p["load_friction_motor_quad"],
        load_friction_external_quad=p["load_friction_external_quad"],
    )
