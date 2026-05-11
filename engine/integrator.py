from pygame.math import Vector2

VectorLike = Vector2 | tuple[float, float] | list[float]
GravityLike = VectorLike | float


def gravity_vector(gravity: GravityLike) -> Vector2:
    if isinstance(gravity, (int, float)):
        return Vector2(0.0, gravity)
    return Vector2(gravity)


def integrate_velocity(
    velocity: Vector2,
    acceleration: VectorLike,
    dt: float,
) -> None:
    velocity += Vector2(acceleration) * dt


def integrate_position(
    position: Vector2,
    velocity: VectorLike,
    dt: float,
) -> None:
    position += Vector2(velocity) * dt


def integrate_semi_implicit_euler(
    position: Vector2,
    velocity: Vector2,
    acceleration: VectorLike,
    dt: float,
) -> None:
    integrate_velocity(velocity, acceleration, dt)
    integrate_position(position, velocity, dt)


def integrate_forces(
    position: Vector2,
    velocity: Vector2,
    force: VectorLike,
    inverse_mass: float,
    dt: float,
    acceleration: VectorLike = (0.0, 0.0),
    gravity: GravityLike = (0.0, 0.0),
) -> None:
    total_acceleration = Vector2(acceleration) + gravity_vector(gravity)
    total_acceleration += Vector2(force) * inverse_mass
    integrate_semi_implicit_euler(position, velocity, total_acceleration, dt)
