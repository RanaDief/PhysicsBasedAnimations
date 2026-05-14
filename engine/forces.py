from pygame.math import Vector2

from .body import VectorLike
from .integrator import GravityLike, gravity_vector


def gravity_force(mass: float, gravity: GravityLike) -> Vector2:
    return gravity_vector(gravity) * mass


def drag_force(velocity: VectorLike, drag_coefficient: float) -> Vector2:
    if drag_coefficient < 0.0:
        raise ValueError("Drag coefficient cannot be negative.")
    return -Vector2(velocity) * drag_coefficient


def wind_force(wind: VectorLike, strength: float = 1.0) -> Vector2:
    return Vector2(wind) * strength


def spring_force(
    position_a: VectorLike,
    position_b: VectorLike,
    rest_length: float,
    stiffness: float,
) -> Vector2:
    if rest_length < 0.0:
        raise ValueError("Rest length cannot be negative.")

    offset = Vector2(position_b) - Vector2(position_a)
    if offset.length_squared() == 0.0:
        return Vector2()

    direction = offset.normalize()
    stretch = offset.length() - rest_length
    return stretch * stiffness * direction


def damping_force(
    velocity_a: VectorLike,
    velocity_b: VectorLike = (0.0, 0.0),
    damping: float = 1.0,
    direction: VectorLike | None = None,
) -> Vector2:
    relative_velocity = Vector2(velocity_a) - Vector2(velocity_b)

    if direction is None:
        return -relative_velocity * damping

    damping_direction = Vector2(direction)
    if damping_direction.length_squared() == 0.0:
        return Vector2()

    damping_direction = damping_direction.normalize()
    return -relative_velocity.dot(damping_direction) * damping * damping_direction


def attraction_force(
    position: VectorLike,
    target: VectorLike,
    strength: float,
    min_distance: float = 1.0,
    max_force: float | None = None,
) -> Vector2:
    return _radial_force(position, target, abs(strength), min_distance, max_force)


def repulsion_force(
    position: VectorLike,
    source: VectorLike,
    strength: float,
    min_distance: float = 1.0,
    max_force: float | None = None,
) -> Vector2:
    return -_radial_force(position, source, abs(strength), min_distance, max_force)


def _radial_force(
    position: VectorLike,
    target: VectorLike,
    strength: float,
    min_distance: float,
    max_force: float | None,
) -> Vector2:
    if min_distance <= 0.0:
        raise ValueError("Minimum distance must be positive.")

    offset = Vector2(target) - Vector2(position)
    distance_squared = max(offset.length_squared(), min_distance * min_distance)
    if offset.length_squared() == 0.0:
        return Vector2()

    force = offset.normalize() * (strength / distance_squared)
    if max_force is not None:
        force = _clamp_magnitude(force, max_force)
    return force


def _clamp_magnitude(vector: Vector2, max_magnitude: float) -> Vector2:
    if max_magnitude < 0.0:
        raise ValueError("Maximum force cannot be negative.")
    if vector.length_squared() <= max_magnitude * max_magnitude:
        return vector
    if vector.length_squared() == 0.0:
        return Vector2()
    return vector.normalize() * max_magnitude
