from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field

import pygame
from pygame.math import Vector2

from engine import (
    AABB,
    Body,
    Bounds,
    CollisionManifold,
    ParticleEmitter,
    SoftBody,
    World,
    detect_circle_aabb_collision,
    resolve_collision,
)
from game.level import Level, ParticleEffectSpec
from game.level_loader import load_level

FPS = 120
MAX_LEVEL = 3
BACKGROUND_COLOR = (16, 19, 25)
GRID_COLOR = (28, 34, 45)
PLATFORM_COLOR = (78, 88, 104)
PLATFORM_EDGE_COLOR = (130, 146, 170)
PLAYER_COLOR = (0, 220, 255)
PLAYER_CORE_COLOR = (226, 250, 255)
CRATE_COLOR = (245, 176, 64)
CRATE_EDGE_COLOR = (255, 220, 132)
SOFT_BODY_FILL = (86, 194, 255)
SOFT_BODY_PARTICLE = (202, 244, 255)
SOFT_BODY_SPRING = (44, 112, 170)
NET_FIXED_COLOR = (255, 95, 95)
BRIDGE_ROPE_COLOR = (178, 142, 92)
BRIDGE_PLANK_COLOR = (142, 92, 48)
BRIDGE_PLANK_EDGE = (222, 177, 104)
SWITCH_OFF_COLOR = (88, 98, 112)
SWITCH_ON_COLOR = (0, 220, 150)
GATE_COLOR = (255, 88, 88)
GATE_OPEN_COLOR = (70, 88, 92)
GOAL_COLOR = (126, 255, 117)
ARM_BASE_COLOR = (88, 116, 150)
ARM_LINK_COLOR = (170, 190, 216)
ARM_JOINT_COLOR = (255, 215, 95)
ARM_GRAB_COLOR = (126, 255, 117)
TEXT_COLOR = (226, 234, 245)
MUTED_TEXT_COLOR = (150, 160, 176)
PLAYER_MOVE_FORCE = 3200.0
PLAYER_MAX_SPEED = 260.0
PLAYER_JUMP_SPEED = 520.0
AIR_CONTROL = 0.65
STATIC_FRICTION = 0.18
SPARK_GRAVITY = (0.0, 520.0)
BRIDGE_LOAD_STIFFNESS = 28.0
BRIDGE_LOAD_DAMPING = 9.5
BRIDGE_MAX_LOAD_SAG = 24.0
ARM_ROTATION_SPEED = 0.035
ARM_GRAB_RANGE = 62.0
ARM_PULL_FORCE = 95000.0


@dataclass(slots=True)
class StaticBox:
    position: Vector2
    half_size: Vector2
    restitution: float = 0.0
    friction: float = STATIC_FRICTION
    velocity: Vector2 = field(default_factory=Vector2)

    @property
    def inv_mass(self) -> float:
        return 0.0


@dataclass(slots=True)
class NetBridge:
    start: Vector2
    columns: int
    spacing: float
    phase: float = 0.0
    load_offsets: list[float] = field(default_factory=list)
    load_velocities: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.load_offsets = [0.0 for _ in range(self.columns)]
        self.load_velocities = [0.0 for _ in range(self.columns)]

    def update(self, dt: float, player: Body) -> None:
        self.phase += dt * 5.0
        player_loads = self._player_loads(player)

        for index, target_offset in enumerate(player_loads):
            offset = self.load_offsets[index]
            velocity = self.load_velocities[index]
            acceleration = (
                (target_offset - offset) * BRIDGE_LOAD_STIFFNESS
                - velocity * BRIDGE_LOAD_DAMPING
            )
            velocity += acceleration * dt
            offset = _clamp(offset + velocity * dt, 0.0, BRIDGE_MAX_LOAD_SAG)
            self.load_velocities[index] = velocity
            self.load_offsets[index] = offset

    def load_offset_at(self, x: float) -> float:
        bridge_x = (x - self.start.x) / self.spacing
        left_index = math.floor(bridge_x)
        right_index = left_index + 1
        if left_index < 0 or right_index >= self.columns:
            return 0.0

        blend = bridge_x - left_index
        return (
            self.load_offsets[left_index] * (1.0 - blend)
            + self.load_offsets[right_index] * blend
        )

    def _player_loads(self, player: Body) -> list[float]:
        loads = [0.0 for _ in range(self.columns)]
        bridge_left = self.start.x - self.spacing * 0.5
        bridge_right = self.start.x + (self.columns - 0.5) * self.spacing
        player_bottom = player.position.y + player.radius
        player_is_on_bridge = (
            bridge_left <= player.position.x <= bridge_right
            and self.start.y + 8.0 <= player_bottom <= self.start.y + 90.0
        )
        if not player_is_on_bridge:
            return loads

        influence_radius = self.spacing * 1.65
        load_depth = _clamp(player.mass / 18.0, 0.5, 2.0) * BRIDGE_MAX_LOAD_SAG
        for index in range(self.columns):
            x = self.start.x + index * self.spacing
            distance = abs(player.position.x - x)
            influence = max(0.0, 1.0 - distance / influence_radius)
            loads[index] = load_depth * influence * influence

        return loads

    def draw(self, surface: pygame.Surface) -> None:
        top_rope: list[Vector2] = []
        bottom_rope: list[Vector2] = []
        middle = (self.columns - 1) * 0.5

        for index in range(self.columns):
            x = self.start.x + index * self.spacing
            sag = 18.0 * (1.0 - abs(index - middle) / middle)
            y = (
                self.start.y
                + sag
                + self.load_offsets[index]
                + 2.0 * math.sin(self.phase + index * 0.65)
            )
            top_rope.append(Vector2(x, y))
            bottom_rope.append(Vector2(x, y + 26.0))

        pygame.draw.lines(surface, BRIDGE_ROPE_COLOR, False, top_rope, 4)
        pygame.draw.lines(surface, BRIDGE_ROPE_COLOR, False, bottom_rope, 4)

        for top, bottom in zip(top_rope, bottom_rope):
            center = (top + bottom) * 0.5
            rect = pygame.Rect(0, 0, 24, 34)
            rect.center = (round(center.x), round(center.y))
            pygame.draw.rect(surface, BRIDGE_PLANK_COLOR, rect, border_radius=2)
            pygame.draw.rect(surface, BRIDGE_PLANK_EDGE, rect, 2, border_radius=2)

        pygame.draw.circle(surface, NET_FIXED_COLOR, top_rope[0], 6)
        pygame.draw.circle(surface, NET_FIXED_COLOR, top_rope[-1], 6)


@dataclass(slots=True)
class ArmState:
    base: Vector2 = field(default_factory=lambda: Vector2(540.0, 400.0))
    shoulder_angle: float = 2.6
    elbow_angle: float = -0.7
    upper_length: float = 160.0
    lower_length: float = 120.0
    grabbing: bool = False

    @property
    def joint(self) -> Vector2:
        return self.base + Vector2(
            math.cos(self.shoulder_angle),
            math.sin(self.shoulder_angle),
        ) * self.upper_length

    @property
    def claw(self) -> Vector2:
        angle = self.shoulder_angle + self.elbow_angle
        direction = Vector2(math.cos(angle), math.sin(angle))
        return self.joint + direction * self.lower_length


@dataclass(slots=True)
class LevelRuntime:
    level: Level
    world: World
    player: Body
    crates: dict[str, Body]
    platforms: list[StaticBox]
    gates: dict[str, StaticBox]
    gate_closed: dict[str, bool]
    soft_bodies: list[SoftBody]
    net_bridges: list[NetBridge]
    arm: ArmState | None
    spark_emitters: list[ParticleEmitter]
    static_surface: pygame.Surface | None = None
    hud_surface: pygame.Surface | None = None
    win_surface: pygame.Surface | None = None
    switch_active: bool = False
    won: bool = False
    grounded: bool = False


def main() -> None:
    pygame.init()
    current_level = _level_number_from_args()
    runtime = _create_runtime(load_level(current_level))
    screen = pygame.display.set_mode(
        (runtime.level.bounds.width, runtime.level.bounds.height)
    )
    pygame.display.set_caption(runtime.level.name)
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 28)
    small_font = pygame.font.Font(None, 22)
    _refresh_runtime_surfaces(runtime, screen, font, small_font)

    running = True
    while running:
        dt = min(clock.tick(FPS) / 1000.0, 1.0 / 30.0)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif (
                event.type == pygame.KEYDOWN
                and event.key == pygame.K_RETURN
                and runtime.won
                and current_level < MAX_LEVEL
            ):
                current_level += 1
                runtime, screen = _load_runtime(
                    current_level,
                    screen,
                    font,
                    small_font,
                )

        _handle_player_input(runtime, pygame.key.get_pressed())
        _update_runtime(runtime, dt)
        _draw_level(screen, runtime, font, small_font)
        pygame.display.flip()

    pygame.quit()


def _load_runtime(
    level_number: int,
    screen: pygame.Surface,
    font: pygame.font.Font,
    small_font: pygame.font.Font,
) -> tuple[LevelRuntime, pygame.Surface]:
    runtime = _create_runtime(load_level(level_number))
    screen = _resize_screen_for_level(screen, runtime.level)
    pygame.display.set_caption(runtime.level.name)
    _refresh_runtime_surfaces(runtime, screen, font, small_font)
    return runtime, screen


def _resize_screen_for_level(
    screen: pygame.Surface,
    level: Level,
) -> pygame.Surface:
    level_size = (level.bounds.width, level.bounds.height)
    if screen.get_size() == level_size:
        return screen
    return pygame.display.set_mode(level_size)


def _refresh_runtime_surfaces(
    runtime: LevelRuntime,
    screen: pygame.Surface,
    font: pygame.font.Font,
    small_font: pygame.font.Font,
) -> None:
    runtime.static_surface = _build_static_surface(screen, runtime)
    runtime.hud_surface = _build_hud_surface(screen, runtime, font, small_font)
    runtime.win_surface = font.render("Power Core delivered", True, GOAL_COLOR)


def _create_runtime(level: Level) -> LevelRuntime:
    player = _create_player(level)
    crates = _create_crates(level)
    platforms = _create_platforms(level)
    gates = _create_gates(level)
    gate_closed = {spec.id: spec.closed for spec in level.gates}
    world = _create_world(level)
    world.add_body(player)
    world.add_bodies(list(crates.values()))

    soft_bodies = _create_soft_bodies(level)
    _add_soft_bodies_to_world(world, level, soft_bodies)

    return LevelRuntime(
        level=level,
        world=world,
        player=player,
        crates=crates,
        platforms=platforms,
        gates=gates,
        gate_closed=gate_closed,
        soft_bodies=soft_bodies,
        net_bridges=_create_net_bridges(level),
        arm=ArmState() if level.id == "level-3" else None,
        spark_emitters=[],
    )


def _create_player(level: Level) -> Body:
    spec = level.player
    return Body(
        spec.position,
        radius=spec.radius,
        mass=spec.mass,
        restitution=spec.restitution,
        friction=spec.friction,
    )


def _create_crates(level: Level) -> dict[str, Body]:
    return {
        spec.id: Body(
            spec.position,
            radius=spec.radius,
            mass=spec.mass,
            restitution=spec.restitution,
            friction=spec.friction,
        )
        for spec in level.rigid_bodies
    }


def _create_platforms(level: Level) -> list[StaticBox]:
    return [
        StaticBox(position=Vector2(spec.position), half_size=Vector2(spec.size) * 0.5)
        for spec in level.platforms
    ]


def _create_gates(level: Level) -> dict[str, StaticBox]:
    return {
        spec.id: StaticBox(
            position=Vector2(spec.position),
            half_size=Vector2(spec.size) * 0.5,
            friction=0.0,
        )
        for spec in level.gates
    }


def _create_world(level: Level) -> World:
    return World(
        bounds=Bounds(0.0, 0.0, level.bounds.width, level.bounds.height),
        gravity=level.gravity,
        floor_friction=420.0,
        collision_iterations=3,
        positional_correction=0.85,
    )


def _create_soft_bodies(level: Level) -> list[SoftBody]:
    return [
        SoftBody(
            center=spec.center,
            particle_count=spec.particle_count,
            radius=spec.radius,
            particle_radius=spec.particle_radius,
            particle_mass=spec.particle_mass,
            spring_stiffness=spec.spring_stiffness,
            spring_damping=spec.spring_damping,
            pressure=spec.pressure,
            restitution=spec.restitution,
        )
        for spec in level.soft_bodies
    ]


def _add_soft_bodies_to_world(
    world: World,
    level: Level,
    soft_bodies: list[SoftBody],
) -> None:
    level_bounds = Bounds(0.0, 0.0, level.bounds.width, level.bounds.height)
    for soft_body, spec in zip(soft_bodies, level.soft_bodies):
        world.add_soft_body(
            soft_body,
            gravity=spec.gravity,
            bounds=level_bounds,
            time_step=spec.time_step,
        )


def _create_net_bridges(level: Level) -> list[NetBridge]:
    if level.id != "level-2":
        return []

    return [
        _create_wobbly_bridge(start=(220.0, 505.0), columns=13, spacing=32.0)
    ]


def _create_wobbly_bridge(
    start: tuple[float, float],
    columns: int,
    spacing: float,
) -> NetBridge:
    return NetBridge(start=Vector2(start), columns=columns, spacing=spacing)


def _handle_player_input(
    runtime: LevelRuntime,
    keys: pygame.key.ScancodeWrapper,
) -> None:
    direction = 0.0
    if keys[pygame.K_LEFT] or keys[pygame.K_a]:
        direction -= 1.0
    if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
        direction += 1.0

    control = 1.0 if runtime.grounded else AIR_CONTROL
    runtime.player.apply_force(
        (direction * PLAYER_MOVE_FORCE * runtime.player.mass * control, 0.0)
    )
    runtime.player.velocity.x = _clamp(
        runtime.player.velocity.x,
        -PLAYER_MAX_SPEED,
        PLAYER_MAX_SPEED,
    )

    wants_jump = keys[pygame.K_SPACE] or keys[pygame.K_UP] or keys[pygame.K_w]
    if wants_jump and runtime.grounded:
        runtime.player.velocity.y = -PLAYER_JUMP_SPEED
        runtime.grounded = False

    _handle_arm_input(runtime, keys)


def _handle_arm_input(runtime: LevelRuntime, keys: pygame.key.ScancodeWrapper) -> None:
    if runtime.arm is None:
        return

    if keys[pygame.K_q]:
        runtime.arm.shoulder_angle -= ARM_ROTATION_SPEED
    if keys[pygame.K_e]:
        runtime.arm.shoulder_angle += ARM_ROTATION_SPEED
    if keys[pygame.K_r]:
        runtime.arm.elbow_angle -= ARM_ROTATION_SPEED
    if keys[pygame.K_f]:
        runtime.arm.elbow_angle += ARM_ROTATION_SPEED

    runtime.arm.shoulder_angle = _clamp(runtime.arm.shoulder_angle, 1.2, 3.0)
    runtime.arm.elbow_angle = _clamp(runtime.arm.elbow_angle, -1.8, 0.9)
    runtime.arm.grabbing = bool(keys[pygame.K_c])
    if runtime.arm.grabbing:
        _pull_nearby_crate(runtime)


def _pull_nearby_crate(runtime: LevelRuntime) -> None:
    if runtime.arm is None:
        return

    for crate in runtime.crates.values():
        offset = runtime.arm.claw - crate.position
        distance = offset.length()
        if 0.0 < distance <= ARM_GRAB_RANGE:
            crate.apply_force(offset.normalize() * ARM_PULL_FORCE)
            crate.velocity *= 0.92


def _update_runtime(runtime: LevelRuntime, dt: float) -> None:
    _update_net_bridges(runtime, dt)
    runtime.world.update(dt)
    _resolve_runtime_collisions(runtime)
    _update_switches(runtime)
    _update_goal_state(runtime)
    _update_spark_emitters(runtime, dt)


def _update_net_bridges(runtime: LevelRuntime, dt: float) -> None:
    for net_bridge in runtime.net_bridges:
        net_bridge.update(dt, runtime.player)
    _sync_bridge_supports(runtime)


def _resolve_runtime_collisions(runtime: LevelRuntime) -> None:
    runtime.grounded = False
    collision_boxes = [*runtime.platforms, *_active_gate_boxes(runtime)]

    for body in [runtime.player, *runtime.crates.values()]:
        for box in collision_boxes:
            collision = _resolve_level_box_collision(body, box)
            if body is runtime.player and _is_ground_collision(collision):
                runtime.grounded = True
                _stop_small_vertical_motion(runtime.player)


def _active_gate_boxes(runtime: LevelRuntime) -> list[StaticBox]:
    return [
        gate for gate_id, gate in runtime.gates.items() if runtime.gate_closed[gate_id]
    ]


def _is_ground_collision(collision: CollisionManifold | None) -> bool:
    return collision is not None and _is_floor_contact(collision.normal)


def _stop_small_vertical_motion(body: Body) -> None:
    if abs(body.velocity.y) < 10.0:
        body.velocity.y = 0.0


def _update_switches(runtime: LevelRuntime) -> None:
    if not runtime.switch_active and _switch_is_pressed(runtime):
        runtime.switch_active = True
        for switch in runtime.level.switches:
            runtime.gate_closed[switch.activates] = False
        _emit_switch_sparks(runtime)


def _update_goal_state(runtime: LevelRuntime) -> None:
    runtime.won = _body_overlaps_circle(
        runtime.player,
        runtime.level.goal.position,
        runtime.level.goal.radius,
    )


def _update_spark_emitters(runtime: LevelRuntime, dt: float) -> None:
    for emitter in runtime.spark_emitters:
        emitter.update(dt)
        emitter.clear_dead()


def _sync_bridge_supports(runtime: LevelRuntime) -> None:
    if not runtime.net_bridges:
        return

    bridge = runtime.net_bridges[0]
    for spec, platform in zip(runtime.level.platforms, runtime.platforms):
        if not spec.id.startswith("soft-bridge-support"):
            continue

        base_position = Vector2(spec.position)
        platform.position.y = base_position.y + bridge.load_offset_at(base_position.x)


def _switch_is_pressed(runtime: LevelRuntime) -> bool:
    for switch in runtime.level.switches:
        body = runtime.crates.get(switch.required_body or "")
        if body is None:
            continue
        switch_box = AABB.from_center(switch.position, switch.size)
        if detect_circle_aabb_collision(body, switch_box):
            return True
    return False


def _resolve_level_box_collision(
    body: Body,
    box: StaticBox,
) -> CollisionManifold | None:
    collision = detect_circle_aabb_collision(
        body,
        AABB.from_center(box.position, box.half_size * 2.0),
    )
    if collision is None:
        return None

    resolve_collision(
        body,
        box,
        collision,
        restitution=0.0,
        friction=STATIC_FRICTION if _is_floor_contact(collision.normal) else 0.0,
        positional_correction=1.0,
    )
    return collision


def _is_floor_contact(normal: Vector2) -> bool:
    return normal.y > 0.6


def _emit_switch_sparks(runtime: LevelRuntime) -> None:
    for effect in runtime.level.particle_effects:
        if effect.trigger != "gate-switch.activated":
            continue
        emitter = _spark_emitter(effect)
        emitter.emit(effect.count)
        runtime.spark_emitters.append(emitter)


def _spark_emitter(effect: ParticleEffectSpec) -> ParticleEmitter:
    x, y = effect.position
    return ParticleEmitter(
        spawn_area=(x - 4.0, y - 4.0, 8.0, 8.0),
        particle_count=0,
        velocity_range=effect.velocity,
        acceleration=SPARK_GRAVITY, 
        radius_range=(2.0, 4.0),
        lifetime_range=effect.lifetime,
        color=effect.color,
        loop=False,
    )


def _draw_level(
    surface: pygame.Surface,
    runtime: LevelRuntime,
    font: pygame.font.Font,
    small_font: pygame.font.Font,
) -> None:
    if runtime.static_surface is None:
        runtime.static_surface = _build_static_surface(surface, runtime)
    surface.blit(runtime.static_surface, (0, 0))
    _draw_switches(surface, runtime)
    _draw_gates(surface, runtime)
    _draw_reactor_arm(surface, runtime)
    for soft_body in runtime.soft_bodies:
        soft_body.draw(
            surface,
            particle_color=SOFT_BODY_PARTICLE,
            spring_color=SOFT_BODY_SPRING,
            fill_color=SOFT_BODY_FILL,
        )
    for net_bridge in runtime.net_bridges:
        net_bridge.draw(surface)
    _draw_body(surface, runtime.player, PLAYER_COLOR, PLAYER_CORE_COLOR)
    for crate in runtime.crates.values():
        _draw_body(surface, crate, CRATE_COLOR, CRATE_EDGE_COLOR)
    for emitter in runtime.spark_emitters:
        emitter.draw(surface)
    _draw_hud(surface, runtime, font, small_font)


def _build_static_surface(
    surface: pygame.Surface,
    runtime: LevelRuntime,
) -> pygame.Surface:
    static_surface = pygame.Surface(surface.get_size()).convert()
    static_surface.fill(BACKGROUND_COLOR)
    _draw_grid(static_surface)
    _draw_platforms(static_surface, runtime)
    _draw_goal(static_surface, runtime)
    return static_surface


def _draw_grid(surface: pygame.Surface) -> None:
    width, height = surface.get_size()
    for x in range(0, width, 40):
        pygame.draw.line(surface, GRID_COLOR, (x, 0), (x, height), 1)
    for y in range(0, height, 40):
        pygame.draw.line(surface, GRID_COLOR, (0, y), (width, y), 1)


def _draw_platforms(surface: pygame.Surface, runtime: LevelRuntime) -> None:
    for spec, platform in zip(runtime.level.platforms, runtime.platforms):
        if spec.kind == "hidden":
            continue
        rect = _box_rect(platform)
        pygame.draw.rect(surface, PLATFORM_COLOR, rect, border_radius=3)
        pygame.draw.rect(surface, PLATFORM_EDGE_COLOR, rect, 2, border_radius=3)


def _draw_switches(surface: pygame.Surface, runtime: LevelRuntime) -> None:
    color = SWITCH_ON_COLOR if runtime.switch_active else SWITCH_OFF_COLOR
    for switch in runtime.level.switches:
        rect = _center_rect(switch.position, switch.size)
        pygame.draw.rect(surface, color, rect, border_radius=3)


def _draw_gates(surface: pygame.Surface, runtime: LevelRuntime) -> None:
    for gate_id, gate in runtime.gates.items():
        rect = _box_rect(gate)
        if runtime.gate_closed[gate_id]:
            pygame.draw.rect(surface, GATE_COLOR, rect, border_radius=4)
        else:
            pygame.draw.rect(surface, GATE_OPEN_COLOR, rect, 2, border_radius=4)


def _draw_goal(surface: pygame.Surface, runtime: LevelRuntime) -> None:
    position = Vector2(runtime.level.goal.position)
    pygame.draw.circle(surface, GOAL_COLOR, position, runtime.level.goal.radius, 3)
    pygame.draw.circle(surface, GOAL_COLOR, position, 5)


def _draw_reactor_arm(surface: pygame.Surface, runtime: LevelRuntime) -> None:
    if runtime.arm is None:
        return

    base = runtime.arm.base
    joint = runtime.arm.joint
    claw = runtime.arm.claw
    claw_color = ARM_GRAB_COLOR if runtime.arm.grabbing else ARM_JOINT_COLOR

    pygame.draw.circle(surface, ARM_BASE_COLOR, base, 18)
    pygame.draw.line(surface, ARM_LINK_COLOR, base, joint, 10)
    pygame.draw.line(surface, ARM_LINK_COLOR, joint, claw, 8)
    pygame.draw.circle(surface, ARM_JOINT_COLOR, joint, 13)
    pygame.draw.circle(surface, claw_color, claw, 9)
    pygame.draw.circle(surface, claw_color, claw, ARM_GRAB_RANGE, 1)
    pygame.draw.line(surface, claw_color, claw, claw + Vector2(18, -10), 4)
    pygame.draw.line(surface, claw_color, claw, claw + Vector2(18, 10), 4)


def _draw_body(
    surface: pygame.Surface,
    body: Body,
    fill_color: tuple[int, int, int],
    edge_color: tuple[int, int, int],
) -> None:
    pygame.draw.circle(surface, fill_color, body.position, body.radius)
    pygame.draw.circle(surface, edge_color, body.position, body.radius, 3)


def _draw_hud(
    surface: pygame.Surface,
    runtime: LevelRuntime,
    font: pygame.font.Font,
    small_font: pygame.font.Font,
) -> None:
    if runtime.hud_surface is None:
        runtime.hud_surface = _build_hud_surface(surface, runtime, font, small_font)
    surface.blit(runtime.hud_surface, (0, 0))
    if runtime.won and runtime.win_surface is not None:
        surface.blit(
            runtime.win_surface,
            (
                surface.get_width() // 2 - runtime.win_surface.get_width() // 2,
                110,
            ),
        )


def _build_hud_surface(
    surface: pygame.Surface,
    runtime: LevelRuntime,
    font: pygame.font.Font,
    small_font: pygame.font.Font,
) -> pygame.Surface:
    hud_surface = pygame.Surface(surface.get_size(), pygame.SRCALPHA).convert_alpha()
    title = font.render(runtime.level.name, True, TEXT_COLOR)
    controls = small_font.render(
        "A/D or arrows move   Space/W jumps",
        True,
        MUTED_TEXT_COLOR,
    )
    hud_surface.blit(title, (18, 16))
    _draw_wrapped_text(
        hud_surface,
        small_font,
        str(runtime.level.metadata["objective"]),
        (18, 46),
        surface.get_width() - 36,
        MUTED_TEXT_COLOR,
    )
    hud_surface.blit(controls, (18, 92))
    if runtime.level.id == "level-3":
        arm_controls = small_font.render(
            "Arm: Q/E shoulder   R/F elbow   Hold C to pull crate",
            True,
            ARM_GRAB_COLOR,
        )
        hud_surface.blit(arm_controls, (18, 116))
    current_level = _level_index(runtime.level)
    if current_level < MAX_LEVEL:
        next_level = small_font.render(
            f"After delivery, press Enter for Level {current_level + 1}",
            True,
            GOAL_COLOR,
        )
        y = 140 if runtime.level.id == "level-3" else 116
        hud_surface.blit(next_level, (18, y))
    return hud_surface


def _draw_wrapped_text(
    surface: pygame.Surface,
    font: pygame.font.Font,
    text: str,
    position: tuple[int, int],
    max_width: int,
    color: tuple[int, int, int],
) -> None:
    x, y = position
    line = ""
    for word in text.split():
        candidate = word if not line else f"{line} {word}"
        if font.size(candidate)[0] <= max_width:
            line = candidate
            continue
        surface.blit(font.render(line, True, color), (x, y))
        y += font.get_linesize()
        line = word
    if line:
        surface.blit(font.render(line, True, color), (x, y))


def _body_overlaps_circle(
    body: Body,
    position: tuple[float, float],
    radius: float,
) -> bool:
    overlap_distance = body.radius + radius
    return body.position.distance_squared_to(Vector2(position)) <= overlap_distance**2


def _box_rect(box: StaticBox) -> pygame.Rect:
    return _center_rect(box.position, box.half_size * 2.0)


def _center_rect(
    position: tuple[float, float] | Vector2,
    size: tuple[float, float] | Vector2,
) -> pygame.Rect:
    center = Vector2(position)
    box_size = Vector2(size)
    return pygame.Rect(
        round(center.x - box_size.x * 0.5),
        round(center.y - box_size.y * 0.5),
        round(box_size.x),
        round(box_size.y),
    )


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def _level_number_from_args() -> int:
    if len(sys.argv) < 2:
        return 1
    try:
        return int(sys.argv[1])
    except ValueError:
        return 1


def _level_index(level: Level) -> int:
    try:
        return int(level.id.rsplit("-", 1)[1])
    except (IndexError, ValueError):
        return 1


if __name__ == "__main__":
    main()
