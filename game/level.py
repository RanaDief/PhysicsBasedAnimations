from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


NumberPair = tuple[float, float]
Color = tuple[int, int, int]


@dataclass(frozen=True, slots=True)
class LevelBounds:
    width: int
    height: int


@dataclass(frozen=True, slots=True)
class SpawnPoint:
    position: NumberPair
    radius: float
    mass: float
    restitution: float
    friction: float


@dataclass(frozen=True, slots=True)
class PlatformSpec:
    id: str
    position: NumberPair
    size: NumberPair
    kind: str = "static"


@dataclass(frozen=True, slots=True)
class RigidBodySpec:
    id: str
    position: NumberPair
    radius: float
    mass: float
    restitution: float
    friction: float


@dataclass(frozen=True, slots=True)
class SwitchSpec:
    id: str
    position: NumberPair
    size: NumberPair
    activates: str
    required_body: str | None = None


@dataclass(frozen=True, slots=True)
class GateSpec:
    id: str
    position: NumberPair
    size: NumberPair
    closed: bool = True


@dataclass(frozen=True, slots=True)
class GoalSpec:
    position: NumberPair
    radius: float


@dataclass(frozen=True, slots=True)
class ParticleEffectSpec:
    id: str
    trigger: str
    position: NumberPair
    color: Color
    count: int
    lifetime: tuple[float, float]
    velocity: tuple[NumberPair, NumberPair]


@dataclass(frozen=True, slots=True)
class Level:
    id: str
    name: str
    bounds: LevelBounds
    gravity: NumberPair
    player: SpawnPoint
    goal: GoalSpec
    platforms: list[PlatformSpec] = field(default_factory=list)
    rigid_bodies: list[RigidBodySpec] = field(default_factory=list)
    switches: list[SwitchSpec] = field(default_factory=list)
    gates: list[GateSpec] = field(default_factory=list)
    particle_effects: list[ParticleEffectSpec] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)
