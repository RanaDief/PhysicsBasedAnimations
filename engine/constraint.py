from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from pygame.math import Vector2

from .body import VectorLike


class PositionBody(Protocol):
    position: Vector2

    @property
    def inv_mass(self) -> float:
        ...


class PositionConstraint(Protocol):
    def solve(self) -> None:
        ...


@dataclass(slots=True)
class DistanceConstraint:
    body_a: PositionBody
    body_b: PositionBody
    rest_length: float
    stiffness: float = 1.0

    @classmethod
    def from_current_distance(
        cls,
        body_a: PositionBody,
        body_b: PositionBody,
        stiffness: float = 1.0,
    ) -> DistanceConstraint:
        return cls(
            body_a=body_a,
            body_b=body_b,
            rest_length=body_a.position.distance_to(body_b.position),
            stiffness=stiffness,
        )

    def solve(self) -> None:
        if self.rest_length < 0.0:
            raise ValueError("Rest length cannot be negative.")

        offset = self.body_b.position - self.body_a.position
        distance = offset.length()
        if distance == 0.0:
            return

        inv_mass_a = _inverse_mass(self.body_a)
        inv_mass_b = _inverse_mass(self.body_b)
        inv_mass_sum = inv_mass_a + inv_mass_b
        if inv_mass_sum == 0.0:
            return

        correction = offset * ((distance - self.rest_length) / distance)
        correction *= _clamp01(self.stiffness)

        self.body_a.position += correction * (inv_mass_a / inv_mass_sum)
        self.body_b.position -= correction * (inv_mass_b / inv_mass_sum)


@dataclass(slots=True)
class PinConstraint:
    body: PositionBody
    target: VectorLike
    stiffness: float = 1.0

    def solve(self) -> None:
        inv_mass = _inverse_mass(self.body)
        if inv_mass == 0.0:
            return

        target = Vector2(self.target)
        self.body.position += (target - self.body.position) * _clamp01(self.stiffness)


@dataclass(slots=True)
class BoundsConstraint:
    body: PositionBody
    min_position: VectorLike
    max_position: VectorLike

    def solve(self) -> None:
        if _inverse_mass(self.body) == 0.0:
            return

        min_position = Vector2(self.min_position)
        max_position = Vector2(self.max_position)
        self.body.position.x = _clamp(
            self.body.position.x,
            min_position.x,
            max_position.x,
        )
        self.body.position.y = _clamp(
            self.body.position.y,
            min_position.y,
            max_position.y,
        )


def solve_constraints(
    constraints: list[PositionConstraint],
    iterations: int = 1,
) -> None:
    if iterations < 0:
        raise ValueError("Constraint iterations cannot be negative.")

    for _ in range(iterations):
        for constraint in constraints:
            constraint.solve()


def update_velocity_from_position(
    body: object,
    previous_position: VectorLike,
    dt: float,
) -> None:
    if dt <= 0.0:
        raise ValueError("Delta time must be positive.")
    if not hasattr(body, "velocity"):
        return

    body.velocity = (body.position - Vector2(previous_position)) / dt


def _inverse_mass(body: PositionBody) -> float:
    if hasattr(body, "inv_mass"):
        return body.inv_mass
    if hasattr(body, "get_inv_mass"):
        return body.get_inv_mass()
    if hasattr(body, "mass"):
        return 0.0 if body.mass == 0.0 else 1.0 / body.mass
    return 1.0


def _clamp01(value: float) -> float:
    return _clamp(value, 0.0, 1.0)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))
