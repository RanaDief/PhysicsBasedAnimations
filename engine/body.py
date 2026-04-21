import math

from pygame.math import Vector2

VectorLike = Vector2 | tuple[float, float] | list[float]


class Body:
    """Minimal circular rigid body used by the physics engine."""

    def __init__(
        self,
        position: VectorLike,
        velocity: VectorLike = (0.0, 0.0),
        radius: float = 20.0,
        mass: float | None = None,
        restitution: float = 1.0,
        friction: float = 0.0,
        acceleration: VectorLike = (0.0, 0.0),
        is_static: bool = False,
    ) -> None:
        self.position = Vector2(position)
        self.velocity = Vector2(velocity)
        self.radius = radius
        self.mass = mass
        self.restitution = restitution
        self.friction = friction
        self.acceleration = Vector2(acceleration)
        self.is_static = is_static
        self.force = Vector2()

        if self.is_static:
            self.mass = math.inf
            return

        if self.mass is None:
            self.mass = float(self.radius)

        if self.mass <= 0:
            raise ValueError("Dynamic bodies must have a positive mass.")

    def get_inv_mass(self) -> float:
        if self.is_static or self.mass is None or math.isinf(self.mass):
            return 0.0
        return 1.0 / self.mass

    def apply_force(self, force: VectorLike) -> None:
        if self.get_inv_mass() == 0.0:
            return
        self.force += Vector2(force)

    def clear_forces(self) -> None:
        self.force.update(0.0, 0.0)

    def integrate(self, dt: float, gravity: VectorLike = (0.0, 0.0)) -> None:
        inverse_mass = self.get_inv_mass()
        if inverse_mass == 0.0:
            return

        total_acceleration = self.acceleration + Vector2(gravity)
        total_acceleration += self.force * inverse_mass

        self.velocity += total_acceleration * dt
        self.position += self.velocity * dt
        self.clear_forces()
