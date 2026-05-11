from __future__ import annotations

from dataclasses import dataclass

from .body import Body, VectorLike
from .collision import Bounds, resolve_all_circle_collisions, resolve_bounds_collision
from .particle import ParticleEmitter, SoftBody


@dataclass(slots=True)
class SoftBodySimulation:
    soft_body: SoftBody
    gravity: VectorLike | float | None = None
    bounds: Bounds | None = None
    time_step: float | None = None


class World:
    """Owns physics objects and advances them in a consistent order."""

    def __init__(
        self,
        bounds: Bounds | None = None,
        gravity: VectorLike = (0.0, 0.0),
        floor_friction: float = 0.0,
        collision_iterations: int = 1,
        positional_correction: float = 1.0,
    ) -> None:
        if collision_iterations < 0:
            raise ValueError("Collision iterations cannot be negative.")

        self.bounds = bounds
        self.gravity = gravity
        self.floor_friction = floor_friction
        self.collision_iterations = collision_iterations
        self.positional_correction = positional_correction
        self.bodies: list[Body] = []
        self.soft_bodies: list[SoftBodySimulation] = []
        self.emitters: list[ParticleEmitter] = []

    def add_body(self, body: Body) -> Body:
        self.bodies.append(body)
        return body

    def add_bodies(self, bodies: list[Body]) -> list[Body]:
        self.bodies.extend(bodies)
        return bodies

    def add_soft_body(
        self,
        soft_body: SoftBody,
        gravity: VectorLike | float | None = None,
        bounds: Bounds | None = None,
        time_step: float | None = None,
    ) -> SoftBody:
        self.soft_bodies.append(
            SoftBodySimulation(
                soft_body=soft_body,
                gravity=gravity,
                bounds=bounds,
                time_step=time_step,
            )
        )
        return soft_body

    def add_emitter(self, emitter: ParticleEmitter) -> ParticleEmitter:
        self.emitters.append(emitter)
        return emitter

    def update(self, dt: float) -> None:
        for emitter in self.emitters:
            emitter.update(dt)

        self._update_bodies(dt)

        for simulation in self.soft_bodies:
            simulation.soft_body.update(
                simulation.time_step if simulation.time_step is not None else dt,
                gravity=simulation.gravity
                if simulation.gravity is not None
                else self.gravity,
                bounds=simulation.bounds if simulation.bounds is not None else self.bounds,
            )

    def _update_bodies(self, dt: float) -> None:
        dynamic_bodies = [body for body in self.bodies if body.inv_mass > 0.0]

        for body in dynamic_bodies:
            body.integrate(dt, gravity=self.gravity)
            self._resolve_bounds(body, dt)

        for _ in range(self.collision_iterations):
            resolve_all_circle_collisions(
                self.bodies,
                positional_correction=self.positional_correction,
            )
            for body in dynamic_bodies:
                self._resolve_bounds(body, dt)

    def _resolve_bounds(self, body: Body, dt: float) -> None:
        if self.bounds is None:
            return

        resolve_bounds_collision(
            body,
            self.bounds,
            dt=dt,
            floor_friction=self.floor_friction,
        )
