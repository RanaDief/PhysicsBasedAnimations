import math

import pygame
from pygame.math import Vector2

from .body import VectorLike
from .collision import Bounds
from .spring import Spring

DEFAULT_GRAVITY = (0.0, 0.5)
DEFAULT_PARTICLE_MASS = 1.0
DEFAULT_PARTICLE_RADIUS = 4.0
DEFAULT_PARTICLE_COUNT = 8
DEFAULT_RESTITUTION = 0.9
DEFAULT_SPRING_STIFFNESS = 0.1
DEFAULT_SPRING_DAMPING = 0.3
DEFAULT_SHAPE_STIFFNESS_SCALE = 0.5
DEFAULT_PRESSURE = 0.1
DEFAULT_SOFT_BODY_RADIUS = 50.0
DEFAULT_MIN_AREA_RATIO = 0.65
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
        is_static: bool = False,
    ) -> None:
        if mass <= 0.0:
            raise ValueError("Particle mass must be positive.")

        self.position = Vector2(position)
        self.velocity = Vector2(velocity)
        self.force = Vector2()
        self.mass = mass
        self.radius = radius
        self.restitution = restitution
        self.is_static = is_static

    def get_inv_mass(self) -> float:
        if self.is_static:
            return 0.0
        return 1.0 / self.mass

    def apply_force(self, force: VectorLike) -> None:
        if self.is_static:
            return
        self.force += Vector2(force)

    def clear_forces(self) -> None:
        self.force.update(0.0, 0.0)

    def integrate(
        self,
        dt: float,
        gravity: VectorLike = DEFAULT_GRAVITY,
        bounds: Bounds | None = None,
    ) -> set[str]:
        if self.is_static:
            self.clear_forces()
            return set()

        acceleration = Vector2(gravity) + self.force * self.get_inv_mass()
        self.velocity += acceleration * dt
        self.position += self.velocity * dt
        self.clear_forces()

        if bounds is None:
            return set()
        return self.resolve_bounds_collision(bounds)

    def resolve_bounds_collision(self, bounds: Bounds) -> set[str]:
        contacts: set[str] = set()

        if self.position.x - self.radius < bounds.min_x:
            self.position.x = bounds.min_x + self.radius
            self.velocity.x = abs(self.velocity.x) * self.restitution
            contacts.add("left")
        elif self.position.x + self.radius > bounds.max_x:
            self.position.x = bounds.max_x - self.radius
            self.velocity.x = -abs(self.velocity.x) * self.restitution
            contacts.add("right")

        if self.position.y - self.radius < bounds.min_y:
            self.position.y = bounds.min_y + self.radius
            self.velocity.y = abs(self.velocity.y) * self.restitution
            contacts.add("top")
        elif self.position.y + self.radius > bounds.max_y:
            self.position.y = bounds.max_y - self.radius
            self.velocity.y = -abs(self.velocity.y) * self.restitution
            contacts.add("bottom")

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
        shape_stiffness: float | None = None,
        shape_damping: float | None = None,
        pressure: float = DEFAULT_PRESSURE,
        min_area_ratio: float = DEFAULT_MIN_AREA_RATIO,
        restitution: float = DEFAULT_RESTITUTION,
    ) -> None:
        if particle_count < 3:
            raise ValueError("A soft body needs at least three particles.")
        if min_area_ratio <= 0.0 or min_area_ratio > 1.0:
            raise ValueError("Minimum area ratio must be between 0 and 1.")

        self.particles: list[Particle] = []
        self.edge_springs: list[Spring] = []
        self.shape_springs: list[Spring] = []
        self.springs: list[Spring] = []
        self.pressure = pressure
        self.min_area_ratio = min_area_ratio

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
            spring = Spring(
                particle,
                next_particle,
                stiffness=spring_stiffness,
                damping=spring_damping,
            )
            self.edge_springs.append(spring)

        support_stiffness = (
            spring_stiffness * DEFAULT_SHAPE_STIFFNESS_SCALE
            if shape_stiffness is None
            else shape_stiffness
        )
        support_damping = spring_damping if shape_damping is None else shape_damping
        self._add_shape_springs(support_stiffness, support_damping)
        self.springs = self.edge_springs + self.shape_springs

        self.initial_area = max(self.area(), MIN_AREA)

    def area(self) -> float:
        total = 0.0
        particle_count = len(self.particles)

        for index, particle in enumerate(self.particles):
            next_particle = self.particles[(index + 1) % particle_count]
            total += particle.position.cross(next_particle.position)

        return abs(total) * 0.5

    def center(self) -> Vector2:
        total = Vector2()
        for particle in self.particles:
            total += particle.position
        return total / len(self.particles)

    def apply_force(self, force: VectorLike) -> None:
        force_per_particle = Vector2(force) / len(self.particles)
        for particle in self.particles:
            particle.apply_force(force_per_particle)

    def update(
        self,
        dt: float,
        gravity: VectorLike = DEFAULT_GRAVITY,
        bounds: Bounds | None = None,
    ) -> set[str]:
        self._apply_spring_and_pressure_forces()

        contacts: set[str] = set()
        for particle in self.particles:
            contacts.update(particle.integrate(dt, gravity, bounds))

        self._preserve_min_area()
        if bounds is not None:
            for particle in self.particles:
                contacts.update(particle.resolve_bounds_collision(bounds))

        return contacts

    def draw(
        self,
        surface,
        particle_color="red",
        spring_color="black",
        fill_color=None,
        show_shape_springs: bool = False,
    ) -> None:
        if fill_color is not None:
            points = [particle.position for particle in self.particles]
            pygame.draw.polygon(surface, _to_color(fill_color), points)

        springs_to_draw = self.springs if show_shape_springs else self.edge_springs
        for spring in springs_to_draw:
            spring.draw(surface, spring_color)

        for particle in self.particles:
            particle.draw(surface, particle_color)

    def _apply_spring_and_pressure_forces(self) -> None:
        current_area = max(self.area(), MIN_AREA)
        pressure_strength = self.pressure * (self.initial_area / current_area)

        for spring in self.shape_springs:
            spring.apply()

        for spring in self.edge_springs:
            spring_force = spring.force()
            pressure_force = pressure_strength * 0.5 * spring.length() * spring.normal()

            spring.particle_a.apply_force(-spring_force + pressure_force)
            spring.particle_b.apply_force(spring_force + pressure_force)

    def _add_shape_springs(self, stiffness: float, damping: float) -> None:
        particle_count = len(self.particles)
        support_steps = {2, particle_count // 2}
        existing_pairs = set()

        for index in range(particle_count):
            existing_pairs.add(self._spring_pair_key(index, index + 1))

        for step in support_steps:
            if step <= 1:
                continue

            for index, particle in enumerate(self.particles):
                other_index = (index + step) % particle_count
                pair_key = self._spring_pair_key(index, other_index)
                if pair_key in existing_pairs:
                    continue

                existing_pairs.add(pair_key)
                self.shape_springs.append(
                    Spring(
                        particle,
                        self.particles[other_index],
                        stiffness=stiffness,
                        damping=damping,
                    )
                )

    def _spring_pair_key(self, index_a: int, index_b: int) -> tuple[int, int]:
        particle_count = len(self.particles)
        key_a = index_a % particle_count
        key_b = index_b % particle_count
        return tuple(sorted((key_a, key_b)))

    def _preserve_min_area(self) -> None:
        current_area = self.area()
        target_area = self.initial_area * self.min_area_ratio
        if current_area >= target_area or current_area <= MIN_AREA:
            return

        scale = (target_area / current_area) ** 0.5
        center = self.center()
        for particle in self.particles:
            offset = particle.position - center
            particle.position = center + offset * scale


def _to_color(color):
    if isinstance(color, str):
        return pygame.Color(color)
    return color
