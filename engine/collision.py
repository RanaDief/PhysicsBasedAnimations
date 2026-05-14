from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Protocol

from pygame.math import Vector2


class CircularBody(Protocol):
    """Small interface that any circular physics body must provide."""

    position: Vector2
    velocity: Vector2
    radius: float
    restitution: float
    friction: float

    @property
    def inv_mass(self) -> float:
        ...


class AABBBody(Protocol):
    """Interface for a movable axis-aligned box body."""

    position: Vector2
    velocity: Vector2
    half_size: Vector2
    restitution: float
    friction: float

    @property
    def inv_mass(self) -> float:
        ...


@dataclass(frozen=True, slots=True)
class Bounds:
    """Axis-aligned rectangle that keeps circular bodies inside the world."""

    min_x: float = 0.0
    min_y: float = 0.0
    max_x: float = 800.0
    max_y: float = 600.0

    @classmethod
    def from_size(
        cls,
        width: float,
        height: float,
        origin: tuple[float, float] = (0.0, 0.0),
    ) -> Bounds:
        min_x, min_y = origin
        return cls(min_x=min_x, min_y=min_y, max_x=min_x + width, max_y=min_y + height)


@dataclass(frozen=True, slots=True)
class AABB:
    """Axis-aligned box represented by minimum and maximum corners."""

    min_x: float
    min_y: float
    max_x: float
    max_y: float

    @classmethod
    def from_center(
        cls,
        center: tuple[float, float] | Vector2,
        size: tuple[float, float] | Vector2,
    ) -> AABB:
        center = Vector2(center)
        half_size = Vector2(size) * 0.5
        return cls(
            min_x=center.x - half_size.x,
            min_y=center.y - half_size.y,
            max_x=center.x + half_size.x,
            max_y=center.y + half_size.y,
        )

    @classmethod
    def from_size(
        cls,
        width: float,
        height: float,
        origin: tuple[float, float] = (0.0, 0.0),
    ) -> AABB:
        min_x, min_y = origin
        return cls(min_x=min_x, min_y=min_y, max_x=min_x + width, max_y=min_y + height)

    @property
    def center(self) -> Vector2:
        return Vector2((self.min_x + self.max_x) * 0.5, (self.min_y + self.max_y) * 0.5)

    @property
    def half_size(self) -> Vector2:
        return Vector2((self.max_x - self.min_x) * 0.5, (self.max_y - self.min_y) * 0.5)


@dataclass(frozen=True, slots=True)
class CollisionManifold:
    """Details about a shape overlap.

    normal points from body_a toward body_b.
    penetration is how far the shapes overlap along the normal.
    """

    normal: Vector2
    penetration: float


def body_aabb(body: AABBBody) -> AABB:
    """Return an AABB snapshot for a box body."""

    half_size = Vector2(body.half_size)
    return AABB(
        min_x=body.position.x - half_size.x,
        min_y=body.position.y - half_size.y,
        max_x=body.position.x + half_size.x,
        max_y=body.position.y + half_size.y,
    )


def detect_circle_collision(
    body_a: CircularBody,
    body_b: CircularBody,
) -> CollisionManifold | None:
    """Return collision details when two circles overlap."""

    center_offset = body_b.position - body_a.position
    combined_radius = body_a.radius + body_b.radius
    squared_distance = center_offset.length_squared()
    squared_touching_distance = combined_radius * combined_radius

    if squared_distance >= squared_touching_distance:
        return None

    if squared_distance == 0.0:
        # Same center: any direction works, so choose a stable horizontal normal.
        return CollisionManifold(normal=Vector2(1.0, 0.0), penetration=combined_radius)

    distance = squared_distance ** 0.5
    collision_normal = center_offset / distance
    overlap_depth = combined_radius - distance
    return CollisionManifold(normal=collision_normal, penetration=overlap_depth)


def detect_aabb_collision(
    box_a: AABB | Bounds,
    box_b: AABB | Bounds,
) -> CollisionManifold | None:
    """Return collision details when two axis-aligned boxes overlap."""

    overlap_x = min(box_a.max_x, box_b.max_x) - max(box_a.min_x, box_b.min_x)
    overlap_y = min(box_a.max_y, box_b.max_y) - max(box_a.min_y, box_b.min_y)

    if overlap_x <= 0.0 or overlap_y <= 0.0:
        return None

    center_offset = _aabb_center(box_b) - _aabb_center(box_a)
    if overlap_x < overlap_y:
        normal = Vector2(1.0 if center_offset.x >= 0.0 else -1.0, 0.0)
        penetration = overlap_x
    else:
        normal = Vector2(0.0, 1.0 if center_offset.y >= 0.0 else -1.0)
        penetration = overlap_y

    return CollisionManifold(normal=normal, penetration=penetration)


def detect_circle_aabb_collision(
    circle: CircularBody,
    box: AABB | Bounds,
) -> CollisionManifold | None:
    """Return collision details when a circle overlaps an axis-aligned box.

    The returned normal is oriented for resolving the circle as body A and the
    box as body B.
    """

    closest = Vector2(
        _clamp(circle.position.x, box.min_x, box.max_x),
        _clamp(circle.position.y, box.min_y, box.max_y),
    )
    offset = closest - circle.position
    squared_distance = offset.length_squared()

    if squared_distance > circle.radius * circle.radius:
        return None

    if squared_distance > 0.0:
        distance = squared_distance ** 0.5
        return CollisionManifold(
            normal=offset / distance,
            penetration=circle.radius - distance,
        )

    return _circle_inside_aabb_collision(circle, box)


def resolve_circle_collision(
    body_a: CircularBody,
    body_b: CircularBody,
    restitution: float | None = None,
    friction: float | None = None,
    positional_correction: float = 1.0,
) -> CollisionManifold | None:
    """Separate two overlapping circles and update their velocities.

    The function returns the detected collision, or None when the circles do
    not overlap.
    """

    collision = detect_circle_collision(body_a, body_b)
    if collision is None:
        return None

    resolve_collision(
        body_a,
        body_b,
        collision,
        restitution=restitution,
        friction=friction,
        positional_correction=positional_correction,
    )
    return collision


def resolve_aabb_collision(
    body_a: AABBBody,
    body_b: AABBBody,
    restitution: float | None = None,
    friction: float | None = None,
    positional_correction: float = 1.0,
) -> CollisionManifold | None:
    """Separate two overlapping AABB bodies and update their velocities."""

    collision = detect_aabb_collision(body_aabb(body_a), body_aabb(body_b))
    if collision is None:
        return None

    resolve_collision(
        body_a,
        body_b,
        collision,
        restitution=restitution,
        friction=friction,
        positional_correction=positional_correction,
    )
    return collision


def resolve_circle_aabb_collision(
    circle: CircularBody,
    box_body: AABBBody,
    restitution: float | None = None,
    friction: float | None = None,
    positional_correction: float = 1.0,
) -> CollisionManifold | None:
    """Separate an overlapping circle and AABB body and update velocities."""

    collision = detect_circle_aabb_collision(circle, body_aabb(box_body))
    if collision is None:
        return None

    resolve_collision(
        circle,
        box_body,
        collision,
        restitution=restitution,
        friction=friction,
        positional_correction=positional_correction,
    )
    return collision


def resolve_collision(
    body_a: CircularBody | AABBBody,
    body_b: CircularBody | AABBBody,
    collision: CollisionManifold,
    restitution: float | None = None,
    friction: float | None = None,
    positional_correction: float = 1.0,
) -> None:
    """Resolve a known collision manifold between two movable bodies."""

    inverse_mass_sum = _combined_inverse_mass(body_a, body_b)
    if inverse_mass_sum == 0.0:
        return

    _separate_bodies(body_a, body_b, collision, positional_correction)
    normal_impulse = _apply_bounce_impulse(
        body_a,
        body_b,
        collision,
        inverse_mass_sum,
        restitution,
    )
    if normal_impulse is None:
        return

    _apply_friction_impulse(
        body_a,
        body_b,
        collision,
        inverse_mass_sum,
        normal_impulse,
        friction,
    )


def resolve_all_circle_collisions(
    bodies: Iterable[CircularBody],
    restitution: float | None = None,
    friction: float | None = None,
    positional_correction: float = 1.0,
) -> int:
    """Resolve every unique circle pair and return how many pairs overlapped."""

    body_list = list(bodies)
    resolved = 0

    for index, body_a in enumerate(body_list):
        for body_b in body_list[index + 1 :]:
            if resolve_circle_collision(
                body_a,
                body_b,
                restitution=restitution,
                friction=friction,
                positional_correction=positional_correction,
            ):
                resolved += 1

    return resolved


def resolve_bounds_collision(
    body: CircularBody,
    bounds: Bounds,
    dt: float = 0.0,
    restitution: float | None = None,
    floor_friction: float | None = None,
) -> set[str]:
    """Keep a circle inside the bounds and return the walls it touched."""

    if body.inv_mass == 0.0:
        return set()

    contacts: set[str] = set()
    bounce = body.restitution if restitution is None else restitution

    if body.position.x - body.radius < bounds.min_x:
        body.position.x = bounds.min_x + body.radius
        body.velocity.x = abs(body.velocity.x) * bounce
        contacts.add("left")
    elif body.position.x + body.radius > bounds.max_x:
        body.position.x = bounds.max_x - body.radius
        body.velocity.x = -abs(body.velocity.x) * bounce
        contacts.add("right")

    if body.position.y - body.radius < bounds.min_y:
        body.position.y = bounds.min_y + body.radius
        body.velocity.y = abs(body.velocity.y) * bounce
        contacts.add("top")
    elif body.position.y + body.radius > bounds.max_y:
        body.position.y = bounds.max_y - body.radius
        body.velocity.y = -abs(body.velocity.y) * bounce
        contacts.add("bottom")

        surface_friction = body.friction if floor_friction is None else floor_friction
        _apply_floor_friction(body, surface_friction, dt)

    return contacts


def _combined_inverse_mass(
    body_a: CircularBody | AABBBody,
    body_b: CircularBody | AABBBody,
) -> float:
    return body_a.inv_mass + body_b.inv_mass


def _separate_bodies(
    body_a: CircularBody | AABBBody,
    body_b: CircularBody | AABBBody,
    manifold: CollisionManifold,
    positional_correction: float,
) -> None:
    inv_mass_sum = body_a.inv_mass + body_b.inv_mass
    if inv_mass_sum == 0.0:
        return

    correction = manifold.normal * (manifold.penetration * positional_correction / inv_mass_sum)
    body_a.position -= correction * body_a.inv_mass
    body_b.position += correction * body_b.inv_mass


def _apply_bounce_impulse(
    body_a: CircularBody | AABBBody,
    body_b: CircularBody | AABBBody,
    manifold: CollisionManifold,
    inv_mass_sum: float,
    restitution: float | None,
) -> float | None:
    """Apply the impulse that makes bodies bounce apart."""

    relative_velocity = body_b.velocity - body_a.velocity
    velocity_along_normal = relative_velocity.dot(manifold.normal)

    if velocity_along_normal >= 0.0:
        return None

    bounce = _collision_restitution(body_a, body_b, restitution)
    impulse_strength = -(1.0 + bounce) * velocity_along_normal / inv_mass_sum
    _apply_impulse(body_a, body_b, manifold.normal * impulse_strength)
    return impulse_strength


def _apply_friction_impulse(
    body_a: CircularBody | AABBBody,
    body_b: CircularBody | AABBBody,
    manifold: CollisionManifold,
    inv_mass_sum: float,
    normal_impulse: float,
    friction: float | None,
) -> None:
    """Apply a sideways impulse that reduces sliding after the bounce."""

    friction_amount = _collision_friction(body_a, body_b, friction)
    if friction_amount <= 0.0:
        return

    relative_velocity = body_b.velocity - body_a.velocity
    tangent_velocity = relative_velocity - manifold.normal * relative_velocity.dot(
        manifold.normal
    )
    if tangent_velocity.length_squared() == 0.0:
        return

    tangent_direction = tangent_velocity.normalize()
    friction_impulse = -relative_velocity.dot(tangent_direction) / inv_mass_sum
    max_friction_impulse = abs(normal_impulse) * friction_amount
    friction_impulse = _clamp(friction_impulse, -max_friction_impulse, max_friction_impulse)

    _apply_impulse(body_a, body_b, tangent_direction * friction_impulse)


def _apply_impulse(
    body_a: CircularBody | AABBBody,
    body_b: CircularBody | AABBBody,
    impulse: Vector2,
) -> None:
    body_a.velocity -= impulse * body_a.inv_mass
    body_b.velocity += impulse * body_b.inv_mass


def _collision_restitution(
    body_a: CircularBody | AABBBody,
    body_b: CircularBody | AABBBody,
    override: float | None,
) -> float:
    if override is not None:
        return override
    return min(body_a.restitution, body_b.restitution)


def _collision_friction(
    body_a: CircularBody | AABBBody,
    body_b: CircularBody | AABBBody,
    override: float | None,
) -> float:
    if override is not None:
        return override
    return (body_a.friction + body_b.friction) * 0.5


def _apply_floor_friction(body: CircularBody, surface_friction: float, dt: float) -> None:
    if surface_friction <= 0.0 or dt <= 0.0 or body.velocity.x == 0.0:
        return

    speed_reduction = surface_friction * dt
    body.velocity.x = _move_toward_zero(body.velocity.x, speed_reduction)


def _move_toward_zero(value: float, amount: float) -> float:
    if abs(value) <= amount:
        return 0.0
    if value > 0.0:
        return value - amount
    return value + amount


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def _aabb_center(box: AABB | Bounds) -> Vector2:
    return Vector2((box.min_x + box.max_x) * 0.5, (box.min_y + box.max_y) * 0.5)


def _circle_inside_aabb_collision(
    circle: CircularBody,
    box: AABB | Bounds,
) -> CollisionManifold:
    distances = {
        "left": circle.position.x - box.min_x,
        "right": box.max_x - circle.position.x,
        "top": circle.position.y - box.min_y,
        "bottom": box.max_y - circle.position.y,
    }
    side = min(distances, key=distances.get)
    distance_to_side = distances[side]

    if side == "left":
        normal = Vector2(1.0, 0.0)
    elif side == "right":
        normal = Vector2(-1.0, 0.0)
    elif side == "top":
        normal = Vector2(0.0, 1.0)
    else:
        normal = Vector2(0.0, -1.0)

    return CollisionManifold(normal=normal, penetration=circle.radius + distance_to_side)
