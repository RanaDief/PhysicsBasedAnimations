import math

import pygame
from pygame.math import Vector2

from .body import VectorLike
from .collision import Bounds
from .spring import Spring

DEFAULT_GRAVITY = 0.5
DEFAULT_PARTICLE_MASS = 1.0
DEFAULT_PARTICLE_RADIUS = 4.0
DEFAULT_PARTICLE_COUNT = 8
DEFAULT_RESTITUTION = 0.9
DEFAULT_SPRING_STIFFNESS = 0.1
DEFAULT_SPRING_DAMPING = 0.3
DEFAULT_PRESSURE = 0.1
DEFAULT_SOFT_BODY_RADIUS = 50.0
MIN_AREA = 1e-9


class Particle:
    """Point mass used by particle effects, springs, and soft bodies."""

    def __init__(
        self,
        position: VectorLike,
        velocity: VectorLike = (0.0, 0.0),
        mass: float = DEFAULT_PARTICLE_MASS,
        radius: float = DEFAULT_PARTICLE_RADIUS,
        restitution: float = DEFAULT_RESTITUTION,
    ) -> None:
        if mass <= 0.0:
            raise ValueError("Particle mass must be positive.")

        self.position = Vector2(position)
        self.velocity = Vector2(velocity)
        self.force = Vector2()
        self.mass = mass
        self.radius = radius
        self.restitution = restitution

    def get_inv_mass(self) -> float:
        return 1.0 / self.mass

    def apply_force(self, force: VectorLike) -> None:
        self.force += Vector2(force)

    def clear_forces(self) -> None:
        self.force.update(0.0, 0.0)

    def integrate(
        self,
        dt: float,
        gravity: VectorLike | float = DEFAULT_GRAVITY,
        bounds: Bounds | None = None,
    ) -> set[str]:
        acceleration = _to_gravity_vector(gravity) + self.force / self.mass
        self.velocity += acceleration * dt
        self.position += self.velocity * dt
        self.clear_forces()

        if bounds is None:
            return set()
        return self.resolve_bounds_collision(bounds)

    def resolve_bounds_collision(self, bounds: Bounds) -> set[str]:
        contacts: set[str] = set()

        if self.position.x >= bounds.max_x - self.radius:
            self.position.x = bounds.max_x - self.radius
            self.velocity.x *= -self.restitution
            contacts.add("right")
        if self.position.x <= bounds.min_x + self.radius:
            self.position.x = bounds.min_x + self.radius
            self.velocity.x *= -self.restitution
            contacts.add("left")

        if self.position.y >= bounds.max_y - self.radius:
            self.position.y = bounds.max_y - self.radius
            self.velocity.y *= -self.restitution
            contacts.add("bottom")
        if self.position.y <= bounds.min_y + self.radius:
            self.position.y = bounds.min_y + self.radius
            self.velocity.y *= -self.restitution
            contacts.add("top")

        return contacts

    def draw(self, surface, color="red") -> None:
        pygame.draw.circle(surface, _to_color(color), self.position, self.radius)


class SoftBody:
    """Closed spring loop that behaves like a simple pressure-filled soft body."""

    def __init__(
        self,
        center: VectorLike,
        particle_count: int = DEFAULT_PARTICLE_COUNT,
        radius: float = DEFAULT_SOFT_BODY_RADIUS,
        particle_mass: float = DEFAULT_PARTICLE_MASS,
        particle_radius: float = DEFAULT_PARTICLE_RADIUS,
        spring_stiffness: float = DEFAULT_SPRING_STIFFNESS,
        spring_damping: float = DEFAULT_SPRING_DAMPING,
        pressure: float = DEFAULT_PRESSURE,
        restitution: float = DEFAULT_RESTITUTION,
    ) -> None:
        if particle_count < 3:
            raise ValueError("A soft body needs at least three particles.")

        self.particles: list[Particle] = []
        self.springs: list[Spring] = []
        self.pressure = pressure

        center_vector = Vector2(center)
        for index in range(particle_count):
            angle = index * (2.0 * math.pi / particle_count)
            offset = Vector2(math.sin(angle), -math.cos(angle)) * radius
            particle = Particle(
                center_vector + offset,
                mass=particle_mass,
                radius=particle_radius,
                restitution=restitution,
            )
            self.particles.append(particle)

        for index, particle in enumerate(self.particles):
            next_particle = self.particles[(index + 1) % particle_count]
            self.springs.append(
                Spring(
                    particle,
                    next_particle,
                    stiffness=spring_stiffness,
                    damping=spring_damping,
                )
            )

        self.initial_area = self.area()

    def area(self) -> float:
        total = 0.0
        particle_count = len(self.particles)

        for index, particle in enumerate(self.particles):
            next_particle = self.particles[(index + 1) % particle_count]
            total += particle.position.cross(next_particle.position)

        return abs(total) * 0.5

    def apply_force(self, force: VectorLike) -> None:
        force_per_particle = Vector2(force) / len(self.particles)
        for particle in self.particles:
            particle.apply_force(force_per_particle)

    def update(
        self,
        dt: float,
        gravity: VectorLike | float = DEFAULT_GRAVITY,
        bounds: Bounds | None = None,
    ) -> set[str]:
        current_area = max(self.area(), MIN_AREA)
        pressure = self.pressure * (self.initial_area / current_area)

        for spring in self.springs:
            length = spring.length()
            normal = spring.normal()
            spring_force = spring.force()
            pressure_force = pressure * 0.5 * length * normal

            spring.particle_a.apply_force(-spring_force + pressure_force)
            spring.particle_b.apply_force(spring_force + pressure_force)

        contacts: set[str] = set()
        for particle in self.particles:
            contacts.update(particle.integrate(dt, gravity, bounds))

        return contacts

    def draw(
        self,
        surface,
        particle_color="red",
        spring_color="black",
        fill_color=None,
    ) -> None:
        if fill_color is not None:
            points = [particle.position for particle in self.particles]
            pygame.draw.polygon(surface, _to_color(fill_color), points)

        for spring in self.springs:
            spring.draw(surface, spring_color)

        for particle in self.particles:
            particle.draw(surface, particle_color)


def _to_color(color):
    if isinstance(color, str):
        return pygame.Color(color)
    return color


def _to_gravity_vector(gravity: VectorLike | float) -> Vector2:
    if isinstance(gravity, (int, float)):
        return Vector2(0.0, gravity)
    return Vector2(gravity)
