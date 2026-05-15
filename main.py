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
NET_PARTICLE_COLOR = (105, 235, 170)
NET_SPRING_COLOR = (190, 220, 245)
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
NET_GRAVITY = 780.0
NET_DAMPING = 0.025
NET_STIFFNESS = 52.0    
NET_PLAYER_FORCE = 9000.0
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
class NetParticle:
    position: Vector2
    velocity: Vector2 = field(default_factory=Vector2)
    is_static: bool = False

    def apply_force(self, force: Vector2, dt: float) -> None:
        if not self.is_static:
            self.velocity += force * dt

    def update(self, dt: float) -> None:
        if self.is_static:
            return
        self.velocity.y += NET_GRAVITY * dt
        self.velocity *= 1.0 - NET_DAMPING
        self.position += self.velocity * dt


@dataclass(slots=True)
class NetSpring:
    particle_a: NetParticle
    particle_b: NetParticle
    rest_length: float

    def apply_force(self, dt: float) -> None:
        offset = self.particle_b.position - self.particle_a.position
        distance = offset.length()
        if distance == 0.0:
            return

        force = offset.normalize() * ((distance - self.rest_length) * NET_STIFFNESS)
        self.particle_a.apply_force(force, dt)
        self.particle_b.apply_force(-force, dt)

    def draw(self, surface: pygame.Surface) -> None:
        pygame.draw.line(
            surface,
            NET_SPRING_COLOR,
            self.particle_a.position,
            self.particle_b.position,
            2,
        )


@dataclass(slots=True)
class NetBridge:
    start: Vector2
    columns: int
    spacing: float
    phase: float = 0.0

    def update(self, dt: float, player: Body) -> None:
        self.phase += dt * 5.0

    def draw(self, surface: pygame.Surface) -> None:
        top_rope: list[Vector2] = []
        bottom_rope: list[Vector2] = []
        middle = (self.columns - 1) * 0.5

        for index in range(self.columns):
            x = self.start.x + index * self.spacing
            sag = 18.0 * (1.0 - abs(index - middle) / middle)
            y = self.start.y + sag + 2.0 * math.sin(self.phase + index * 0.65)
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
        return self.joint + Vector2(math.cos(angle), math.sin(angle)) * self.lower_length


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
    runtime.static_surface = _build_static_surface(screen, runtime)
    runtime.hud_surface = _build_hud_surface(screen, runtime, font, small_font)
    runtime.win_surface = font.render("Power Core delivered", True, GOAL_COLOR)

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
    if screen.get_size() != (runtime.level.bounds.width, runtime.level.bounds.height):
        screen = pygame.display.set_mode(
            (runtime.level.bounds.width, runtime.level.bounds.height)
        )
    pygame.display.set_caption(runtime.level.name)
    runtime.static_surface = _build_static_surface(screen, runtime)
    runtime.hud_surface = _build_hud_surface(screen, runtime, font, small_font)
    runtime.win_surface = font.render("Power Core delivered", True, GOAL_COLOR)
    return runtime, screen


def _create_runtime(level: Level) -> LevelRuntime:
    player_spec = level.player
    player = Body(
        player_spec.position,
        radius=player_spec.radius,
        mass=player_spec.mass,
        restitution=player_spec.restitution,
        friction=player_spec.friction,
    )
    crates = {
        spec.id: Body(
            spec.position,
            radius=spec.radius,
            mass=spec.mass,
            restitution=spec.restitution,
            friction=spec.friction,
        )
        for spec in level.rigid_bodies
    }
    platforms = [
        StaticBox(position=Vector2(spec.position), half_size=Vector2(spec.size) * 0.5)
        for spec in level.platforms
    ]
    gates = {
        spec.id: StaticBox(
            position=Vector2(spec.position),
            half_size=Vector2(spec.size) * 0.5,
            friction=0.0,
        )
        for spec in level.gates
    }
    gate_closed = {spec.id: spec.closed for spec in level.gates}
    world = World(
        bounds=Bounds(0.0, 0.0, level.bounds.width, level.bounds.height),
        gravity=level.gravity,
        floor_friction=420.0,
        collision_iterations=3,
        positional_correction=0.85,
    )
    world.add_body(player)
    world.add_bodies(list(crates.values()))
    soft_bodies = [
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
    for soft_body, spec in zip(soft_bodies, level.soft_bodies):
        world.add_soft_body(
            soft_body,
            gravity=spec.gravity,
            bounds=Bounds(0.0, 0.0, level.bounds.width, level.bounds.height),
            time_step=spec.time_step,
        )
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


def _net_spring(particle_a: NetParticle, particle_b: NetParticle) -> NetSpring:
    return NetSpring(
        particle_a=particle_a,
        particle_b=particle_b,
        rest_length=particle_a.position.distance_to(particle_b.position),
    )


def _handle_player_input(runtime: LevelRuntime, keys: pygame.key.ScancodeWrapper) -> None:
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

    if (keys[pygame.K_SPACE] or keys[pygame.K_UP] or keys[pygame.K_w]) and runtime.grounded:
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
    runtime.world.update(dt)
    runtime.grounded = False

    active_gate_boxes = [
        gate for gate_id, gate in runtime.gates.items() if runtime.gate_closed[gate_id]
    ]
    for body in [runtime.player, *runtime.crates.values()]:
        for box in [*runtime.platforms, *active_gate_boxes]:
            collision = _resolve_level_box_collision(body, box)
            if body is runtime.player and collision is not None and _is_floor_contact(collision.normal):
                runtime.grounded = True
                if abs(runtime.player.velocity.y) < 10.0:   
                    runtime.player.velocity.y = 0.0

    if not runtime.switch_active and _switch_is_pressed(runtime):
        runtime.switch_active = True
        for switch in runtime.level.switches:
            runtime.gate_closed[switch.activates] = False
        _emit_switch_sparks(runtime)

    runtime.won = _body_overlaps_circle(runtime.player, runtime.level.goal.position, runtime.level.goal.radius)
    for net_bridge in runtime.net_bridges:
        net_bridge.update(dt, runtime.player)
    for emitter in runtime.spark_emitters:
        emitter.update(dt)
        emitter.clear_dead()


def _switch_is_pressed(runtime: LevelRuntime) -> bool:
    for switch in runtime.level.switches:
        body = runtime.crates.get(switch.required_body or "")
        if body is None:
            continue
        if detect_circle_aabb_collision(body, AABB.from_center(switch.position, switch.size)):
            return True
    return False


def _resolve_level_box_collision(body: Body, box: StaticBox):
    collision = detect_circle_aabb_collision(body, AABB.from_center(box.position, box.half_size * 2.0))
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
    controls = small_font.render("A/D or arrows move   Space/W jumps", True, MUTED_TEXT_COLOR)
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
        hud_surface.blit(next_level, (18, 140 if runtime.level.id == "level-3" else 116))
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
    return body.position.distance_squared_to(Vector2(position)) <= (body.radius + radius) ** 2


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
