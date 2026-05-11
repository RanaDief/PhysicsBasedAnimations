import math
import random
from dataclasses import dataclass
from typing import Callable

import pygame
from pygame.math import Vector2

from .body import VectorLike
from .collision import Bounds
from .integrator import integrate_forces, integrate_semi_implicit_euler
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
ColorLike = pygame.Color | str | tuple[int, int, int] | tuple[int, int, int, int]
Range = tuple[float, float]
VectorRange = tuple[VectorLike, VectorLike]


@dataclass
class VisualParticle:
    """Lightweight render particle owned by a ParticleEmitter."""

    position: Vector2
    velocity: Vector2
    acceleration: Vector2
    radius: float
    color: ColorLike
    lifetime: float
    age: float = 0.0

    @property
    def alive(self) -> bool:
        return self.age < self.lifetime

    def update(self, dt: float) -> None:
        integrate_semi_implicit_euler(
            self.position,
            self.velocity,
            self.acceleration,
            dt,
        )
        self.age += dt

    def draw(self, surface) -> None:
        pygame.draw.circle(surface, _to_color(self.color), self.position, self.radius)


class ParticleEmitter:
    """Configurable visual particle system for effects such as rain, sparks, or dust."""

    def __init__(
        self,
        spawn_area: pygame.Rect | tuple[float, float, float, float],
        particle_count: int,
        velocity_range: VectorRange,
        acceleration: VectorLike = (0.0, 0.0),
        radius_range: Range = (2.0, 5.0),
        lifetime_range: Range = (1.0, 3.0),
        color: ColorLike | Callable[[random.Random], ColorLike] = "white",
        bounds: pygame.Rect | tuple[float, float, float, float] | None = None,
        loop: bool = True,
        seed: int | None = None,
    ) -> None:
        if particle_count < 0:
            raise ValueError("Particle count cannot be negative.")
        _validate_range(radius_range, "Particle radius range")
        _validate_range(lifetime_range, "Particle lifetime range")

        self.spawn_area = pygame.Rect(spawn_area)
        self.particle_count = particle_count
        self.velocity_range = velocity_range
        self.acceleration = Vector2(acceleration)
        self.radius_range = radius_range
        self.lifetime_range = lifetime_range
        self.color = color
        self.bounds = pygame.Rect(bounds) if bounds is not None else None
        self.loop = loop
        self.random = random.Random(seed)
        self.particles: list[VisualParticle] = [
            self._create_particle(randomize_age=True) for _ in range(particle_count)
        ]

    def update(self, dt: float) -> None:
        for index, particle in enumerate(self.particles):
            particle.update(dt)
            if self._should_respawn(particle):
                if self.loop:
                    self.particles[index] = self._create_particle()
                else:
                    particle.age = particle.lifetime

    def draw(self, surface) -> None:
        for particle in self.particles:
            if particle.alive:
                particle.draw(surface)

    def emit(self, count: int) -> None:
        """Add a burst of particles without changing the steady emitter count."""
        if count < 0:
            raise ValueError("Emit count cannot be negative.")

        self.particles.extend(self._create_particle() for _ in range(count))

    def clear_dead(self) -> None:
        self.particles = [particle for particle in self.particles if particle.alive]

    def _create_particle(self, randomize_age: bool = False) -> VisualParticle:
        lifetime = self.random.uniform(*self.lifetime_range)
        age = self.random.uniform(0.0, lifetime) if randomize_age else 0.0
        return VisualParticle(
            position=self._random_position(),
            velocity=self._random_vector(self.velocity_range),
            acceleration=Vector2(self.acceleration),
            radius=self.random.uniform(*self.radius_range),
            color=self._random_color(),
            lifetime=lifetime,
            age=age,
        )

    def _random_position(self) -> Vector2:
        return Vector2(
            self.random.uniform(self.spawn_area.left, self.spawn_area.right),
            self.random.uniform(self.spawn_area.top, self.spawn_area.bottom),
        )

    def _random_vector(self, vector_range: VectorRange) -> Vector2:
        min_vector = Vector2(vector_range[0])
        max_vector = Vector2(vector_range[1])
        return Vector2(
            self.random.uniform(min_vector.x, max_vector.x),
            self.random.uniform(min_vector.y, max_vector.y),
        )

    def _random_color(self) -> ColorLike:
        if callable(self.color):
            return self.color(self.random)
        return self.color

    def _should_respawn(self, particle: VisualParticle) -> bool:
        if not particle.alive:
            return True
        if self.bounds is None:
            return False
        return not self.bounds.collidepoint(particle.position)


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
        integrate_forces(
            self.position,
            self.velocity,
            self.force,
            self.get_inv_mass(),
            dt,
            gravity=gravity,
        )
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


def _validate_range(value_range: Range, label: str) -> None:
    if value_range[0] > value_range[1]:
        raise ValueError(f"{label} minimum cannot be greater than maximum.")
