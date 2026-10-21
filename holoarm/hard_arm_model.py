"""Original hard-arm mechanics for comparison with the supplied soft CAD.

The X2-inspired artwork is illustrative. Its primitive collision geometry,
relative mass distribution, rotor layout and welded arms come from the legacy
model; they are not measured Skydio hardware specifications.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import math
from pathlib import Path

from cad_model import load_scene
from legacy_collision_study import build_model as build_legacy_model
from legacy_collision_study import load_config as load_legacy_config


HERE = Path(__file__).resolve().parent
ARMS = ("fl", "fr", "rl", "rr")
SHARED_PARAMETERS = (
    "mass_kg", "timestep_s", "altitude_m", "max_motor_thrust_n",
    "motor_time_constant_s", "contact_time_constant_s", "contact_damping_ratio",
)


def _wall_dimensions(cfg):
    scene = load_scene()["impact_wall"]
    wall = {
        "wall_front_x_m": cfg.get("wall_front_x_m", scene["front_x_m"]),
        "wall_thickness_m": cfg.get("wall_thickness_m", scene["thickness_m"]),
        "wall_width_m": cfg.get("wall_width_m", scene["width_m"]),
        "wall_height_m": cfg.get("wall_height_m", scene["height_m"]),
    }
    for key, value in wall.items():
        if not math.isfinite(value) or (key != "wall_front_x_m" and value <= 0):
            raise ValueError(f"{key} must be finite" + (" and positive" if key != "wall_front_x_m" else ""))
    return wall


def build_hard_arm_model(cfg, width=640, height=540, environment=None,
                         model_transform=None):
    """Build the previous hard frame with the current trial's shared settings.

    ``motor_time_constant_s`` is consumed by the shared Trial controller, not
    encoded as a second actuator delay here. The legacy rotor gears and guard
    radius are retained; CAD constitutive parameters do not enter this model.
    """
    legacy_cfg = load_legacy_config()
    for key in SHARED_PARAMETERS:
        if key in cfg:
            if not math.isfinite(cfg[key]) or cfg[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
            legacy_cfg[key] = cfg[key]
    wall = _wall_dimensions(cfg)

    # Clone caller geometry: the legacy builder attaches the supplied elements
    # directly, while this bridge adjusts their collision masks and dimensions.
    arena = deepcopy(environment) if environment is not None else None

    def adapt(root):
        root.set("model", "Original hard-arm frame | X2-inspired artwork")
        world = root.find("worldbody")
        impact_wall = world.find("geom[@name='impact_wall']")
        if impact_wall is not None:
            thickness = wall["wall_thickness_m"]
            height_m = wall["wall_height_m"]
            impact_wall.set("pos", f"{wall['wall_front_x_m'] + thickness/2} 0 {height_m/2}")
            impact_wall.set("size", f"{thickness/2} {wall['wall_width_m']/2} {height_m/2}")
            impact_wall.set("condim", "1")
            # The original orange stripe is decorative and has no contacts.
            for geom in world.findall("geom"):
                if geom.get("name") is None and geom.get("type") == "box":
                    geom.set("pos", f"{wall['wall_front_x_m'] - .001} 0 {height_m/2}")
                    geom.set("size", f".002 {wall['wall_width_m']/2} .012")
        if arena is not None:
            for elem in arena:
                if elem.tag == "geom" and elem.get("contype", "1") != "0":
                    elem.set("contype", "1")
                    elem.set("conaffinity", "2")
                    elem.set("condim", "1")
        for geom in root.iter("geom"):
            name = geom.get("name", "")
            if any(name.startswith(f"{arm}_guard_") for arm in ARMS):
                geom.set("condim", "1")
                # These existing colliders were hidden by the X2 visual kit.
                # Show the actual contact envelope without adding any geometry.
                geom.set("rgba", ".24 .27 .30 1")
        for motor in root.find("actuator"):
            motor.set("ctrllimited", "true")
            motor.set("ctrlrange", f"0 {legacy_cfg['max_motor_thrust_n']}")
        if model_transform is not None:
            model_transform(root)

    return build_legacy_model(
        "rigid", legacy_cfg, width=width, height=height, rigid_visual="x2",
        environment=arena, model_transform=adapt,
    )


def hard_arm_provenance(cfg=None):
    """Describe the distinct hard model without attributing CAD data to it."""
    legacy_cfg = load_legacy_config()
    sources = (
        "models/holoarm_physics.xml", "models/rigid_x2_visual.xml",
        "models/legacy_wall_collision_config.json",
    )
    result = {
        "model": "Original generic hard-arm frame with X2-inspired artwork",
        "geometry_model": "legacy_hard_arm",
        "mechanics": "Original primitive collision geometry and relative mass distribution; four welded free arms",
        "visuals": "Original X2-inspired artwork plus visible physical guard rings; no CAD skins",
        "guard_radius_m": legacy_cfg["guard_radius_m"],
        "guard_capsule_radius_m": .003,
        "guard_segments_per_rotor": 20,
        "yaw_torque_per_thrust_m": .02,
        "contact_model": "normal_only_ideal_slip_guard_wall",
        "source_sha256": {
            name: hashlib.sha256((HERE/name).read_bytes()).hexdigest()
            for name in sources
        },
        "scope": "Illustrative generic hard-arm mechanics, not measured Skydio geometry, inertia or fracture data. No CAD stiffness or rheology is applied.",
    }
    if cfg is not None:
        result["shared_trial_parameters"] = {
            key: cfg.get(key, legacy_cfg[key]) for key in SHARED_PARAMETERS
        }
        result["wall_geometry"] = _wall_dimensions(cfg)
    return result
