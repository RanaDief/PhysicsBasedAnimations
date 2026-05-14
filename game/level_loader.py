from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .level import (
    GateSpec,
    GoalSpec,
    Level,
    LevelBounds,
    ParticleEffectSpec,
    PlatformSpec,
    RigidBodySpec,
    SoftBodySpec,
    SpawnPoint,
    SwitchSpec,
)


LEVELS_DIR = Path(__file__).resolve().parent.parent / "levels"


def load_level(level_number: int, levels_dir: Path = LEVELS_DIR) -> Level:
    if level_number <= 0:
        raise ValueError("Level number must be positive.")

    return load_level_file(levels_dir / f"level{level_number}.json")


def load_level_file(path: str | Path) -> Level:
    level_path = Path(path)
    with level_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    return level_from_dict(data)


def level_from_dict(data: dict[str, Any]) -> Level:
    _require_keys(data, "level", ["id", "name", "bounds", "gravity", "player", "goal"])

    return Level(
        id=str(data["id"]),
        name=str(data["name"]),
        bounds=_bounds(data["bounds"]),
        gravity=_number_pair(data["gravity"], "gravity"),
        player=_spawn_point(data["player"]),
        goal=_goal(data["goal"]),
        platforms=[_platform(item) for item in data.get("platforms", [])],
        rigid_bodies=[_rigid_body(item) for item in data.get("rigid_bodies", [])],
        switches=[_switch(item) for item in data.get("switches", [])],
        gates=[_gate(item) for item in data.get("gates", [])],
        particle_effects=[
            _particle_effect(item) for item in data.get("particle_effects", [])
        ],
        soft_bodies=[_soft_body(item) for item in data.get("soft_bodies", [])],
        metadata=dict(data.get("metadata", {})),
    )


def _bounds(data: dict[str, Any]) -> LevelBounds:
    _require_keys(data, "bounds", ["width", "height"])
    return LevelBounds(width=int(data["width"]), height=int(data["height"]))


def _spawn_point(data: dict[str, Any]) -> SpawnPoint:
    _require_keys(data, "player", ["position", "radius", "mass", "restitution", "friction"])
    return SpawnPoint(
        position=_number_pair(data["position"], "player.position"),
        radius=float(data["radius"]),
        mass=float(data["mass"]),
        restitution=float(data["restitution"]),
        friction=float(data["friction"]),
    )


def _platform(data: dict[str, Any]) -> PlatformSpec:
    _require_keys(data, "platform", ["id", "position", "size"])
    return PlatformSpec(
        id=str(data["id"]),
        position=_number_pair(data["position"], f"platform.{data['id']}.position"),
        size=_number_pair(data["size"], f"platform.{data['id']}.size"),
        kind=str(data.get("kind", "static")),
    )


def _rigid_body(data: dict[str, Any]) -> RigidBodySpec:
    _require_keys(
        data,
        "rigid_body",
        ["id", "position", "radius", "mass", "restitution", "friction"],
    )
    return RigidBodySpec(
        id=str(data["id"]),
        position=_number_pair(data["position"], f"rigid_body.{data['id']}.position"),
        radius=float(data["radius"]),
        mass=float(data["mass"]),
        restitution=float(data["restitution"]),
        friction=float(data["friction"]),
    )


def _switch(data: dict[str, Any]) -> SwitchSpec:
    _require_keys(data, "switch", ["id", "position", "size", "activates"])
    return SwitchSpec(
        id=str(data["id"]),
        position=_number_pair(data["position"], f"switch.{data['id']}.position"),
        size=_number_pair(data["size"], f"switch.{data['id']}.size"),
        activates=str(data["activates"]),
        required_body=data.get("required_body"),
    )


def _gate(data: dict[str, Any]) -> GateSpec:
    _require_keys(data, "gate", ["id", "position", "size"])
    return GateSpec(
        id=str(data["id"]),
        position=_number_pair(data["position"], f"gate.{data['id']}.position"),
        size=_number_pair(data["size"], f"gate.{data['id']}.size"),
        closed=bool(data.get("closed", True)),
    )


def _goal(data: dict[str, Any]) -> GoalSpec:
    _require_keys(data, "goal", ["position", "radius"])
    return GoalSpec(
        position=_number_pair(data["position"], "goal.position"),
        radius=float(data["radius"]),
    )


def _particle_effect(data: dict[str, Any]) -> ParticleEffectSpec:
    _require_keys(
        data,
        "particle_effect",
        ["id", "trigger", "position", "color", "count", "lifetime", "velocity"],
    )
    return ParticleEffectSpec(
        id=str(data["id"]),
        trigger=str(data["trigger"]),
        position=_number_pair(data["position"], f"particle_effect.{data['id']}.position"),
        color=_color(data["color"], f"particle_effect.{data['id']}.color"),
        count=int(data["count"]),
        lifetime=_number_pair(data["lifetime"], f"particle_effect.{data['id']}.lifetime"),
        velocity=(
            _number_pair(data["velocity"][0], f"particle_effect.{data['id']}.velocity[0]"),
            _number_pair(data["velocity"][1], f"particle_effect.{data['id']}.velocity[1]"),
        ),
    )


def _soft_body(data: dict[str, Any]) -> SoftBodySpec:
    _require_keys(
        data,
        "soft_body",
        [
            "id",
            "center",
            "particle_count",
            "radius",
            "particle_radius",
            "particle_mass",
            "spring_stiffness",
            "spring_damping",
            "pressure",
            "restitution",
            "gravity",
            "time_step",
        ],
    )
    return SoftBodySpec(
        id=str(data["id"]),
        center=_number_pair(data["center"], f"soft_body.{data['id']}.center"),
        particle_count=int(data["particle_count"]),
        radius=float(data["radius"]),
        particle_radius=float(data["particle_radius"]),
        particle_mass=float(data["particle_mass"]),
        spring_stiffness=float(data["spring_stiffness"]),
        spring_damping=float(data["spring_damping"]),
        pressure=float(data["pressure"]),
        restitution=float(data["restitution"]),
        gravity=_number_pair(data["gravity"], f"soft_body.{data['id']}.gravity"),
        time_step=float(data["time_step"]),
    )


def _number_pair(value: Any, label: str) -> tuple[float, float]:
    if not isinstance(value, list | tuple) or len(value) != 2:
        raise ValueError(f"{label} must contain two numbers.")
    return (float(value[0]), float(value[1]))


def _color(value: Any, label: str) -> tuple[int, int, int]:
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{label} must contain three color channels.")
    return (int(value[0]), int(value[1]), int(value[2]))


def _require_keys(data: dict[str, Any], label: str, keys: list[str]) -> None:
    missing = [key for key in keys if key not in data]
    if missing:
        raise ValueError(f"{label} is missing required keys: {', '.join(missing)}")
