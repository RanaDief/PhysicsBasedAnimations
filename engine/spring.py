import pygame
from pygame.math import Vector2


class Spring:
    """Elastic connection between two soft-body particles."""

    def __init__(
        self,
        particle_a,
        particle_b,
        stiffness: float = 0.1,
        damping: float = 0.3,
        rest_length: float | None = None,
    ) -> None:
        self.particle_a = particle_a
        self.particle_b = particle_b
        self.stiffness = stiffness
        self.damping = damping

        if rest_length is None:
            self.rest_length = self.length()
        else:
            self.rest_length = rest_length

    def length(self) -> float:
        return self.particle_a.position.distance_to(self.particle_b.position)

    def direction(self) -> Vector2:
        offset = self.particle_a.position - self.particle_b.position
        if offset.length_squared() == 0.0:
            return Vector2()
        return offset.normalize()

    def normal(self) -> Vector2:
        offset = self.particle_a.position - self.particle_b.position
        if offset.length_squared() == 0.0:
            return Vector2()

        normal = Vector2(-offset.y, offset.x)
        return normal.normalize()

    def force(self) -> Vector2:
        direction = self.direction()
        if direction.length_squared() == 0.0:
            return Vector2()

        stretch = self.length() - self.rest_length
        spring_force = stretch * self.stiffness * direction

        relative_velocity = self.particle_a.velocity - self.particle_b.velocity
        damping_force = relative_velocity.dot(direction) * self.damping * direction

        return spring_force + damping_force

    def apply(self) -> None:
        spring_force = self.force()
        self.particle_a.apply_force(-spring_force)
        self.particle_b.apply_force(spring_force)

    def draw(self, surface, color="black", width: int = 1) -> None:
        pygame.draw.line(
            surface,
            _to_color(color),
            self.particle_a.position,
            self.particle_b.position,
            width,
        )


def _to_color(color):
    if isinstance(color, str):
        return pygame.Color(color)
    return color
