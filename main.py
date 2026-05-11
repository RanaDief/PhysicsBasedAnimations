import pygame

from engine import (
    Body,
    Bounds,
    CCDInverseKinematicsSolver,
    ForwardKinematicsChain,
    ParticleEmitter,
    SoftBody,
    World,
)

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60
BACKGROUND_COLOR = (18, 20, 24)
PANEL_LINE_COLOR = (72, 80, 94)
FK_BASE_COLOR = (48, 118, 255)
FK_LINK_1_COLOR = (255, 88, 88)
FK_LINK_2_COLOR = (84, 220, 120)
IK_JOINT_COLOR = (0, 200, 255)
IK_LINK_COLOR = (245, 248, 255)
IK_END_COLOR = (255, 64, 64)
IK_TARGET_COLOR = (255, 232, 84)
BASE_RADIUS = 10
LINK_WIDTH = 3
FK_LINK_LENGTHS = [65.0, 100.0]
IK_LINK_LENGTHS = [150.0, 100.0]
TARGET_SPEED = 5.0
SOFT_BODY_GRAVITY = 0.5
SOFT_BODY_DT = 0.5
SOFT_BODY_FILL = (244, 132, 132)
SOFT_BODY_PARTICLE = (180, 36, 36)
SOFT_BODY_SPRING = (235, 238, 245)
RIGID_BODY_COLORS = [(255, 192, 74), (96, 208, 255), (132, 236, 128)]
RIGID_BODY_STATIC_COLOR = (100, 108, 122)
RIGID_BODY_OUTLINE = (245, 248, 255)
RIGID_BODY_GRAVITY = (0.0, 900.0)
RIGID_BODY_FRICTION = 420.0
RAIN_PARTICLE_COLOR = (0, 150, 170)
RAIN_PARTICLE_COUNT = 100
BALL_BOUNCINESS = 0.9
BALL_PRESSURE = 0.1
BALL_SPRING_STIFFNESS = 0.1
BALL_SPRING_DAMPING = 0.3


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Soft Body")
    clock = pygame.time.Clock()

    fk_base = (SCREEN_WIDTH * 0.25, SCREEN_HEIGHT * 0.25)
    fk_chain = ForwardKinematicsChain(base=fk_base, link_lengths=FK_LINK_LENGTHS)
    ik_base = (SCREEN_WIDTH * 0.68, SCREEN_HEIGHT * 0.25)
    ik_chain = ForwardKinematicsChain(base=ik_base, link_lengths=IK_LINK_LENGTHS)
    ik_solver = CCDInverseKinematicsSolver(ik_chain, iterations=10)
    ik_angles = [0.5, 0.5]
    ik_target = [SCREEN_WIDTH * 0.78, SCREEN_HEIGHT * 0.25]

    rigid_body_bounds = Bounds(0, SCREEN_HEIGHT * 0.5, SCREEN_WIDTH * 0.5, SCREEN_HEIGHT)
    rigid_bodies = [
        Body((95, 340), velocity=(180, 20), radius=22, restitution=0.85, friction=0.0),
        Body((185, 330), velocity=(-120, 0), radius=28, restitution=0.8, friction=0.0),
        Body((305, 350), velocity=(-80, -30), radius=18, restitution=0.9, friction=0.0),
        Body((205, 540), radius=36, restitution=0.7, is_static=True),
    ]

    soft_body_bounds = Bounds(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT)
    soft_body = SoftBody(
        center=(SCREEN_WIDTH * 0.5, SCREEN_HEIGHT * 0.45),
        particle_count=8,
        radius=50,
        particle_radius=4,
        spring_stiffness=BALL_SPRING_STIFFNESS,
        spring_damping=BALL_SPRING_DAMPING,
        pressure=BALL_PRESSURE,
        restitution=BALL_BOUNCINESS,
    )
    particle_system = ParticleEmitter(
        spawn_area=(0, -20, SCREEN_WIDTH, SCREEN_HEIGHT + 20),
        particle_count=RAIN_PARTICLE_COUNT,
        velocity_range=((0.0, 120.0), (0.0, 360.0)),
        acceleration=(0.0, 40.0),
        radius_range=(3.0, 6.0),
        lifetime_range=(1.0, 5.0),
        color=RAIN_PARTICLE_COLOR,
        bounds=(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT),
        loop=True,
    )
    world = World(
        bounds=rigid_body_bounds,
        gravity=RIGID_BODY_GRAVITY,
        floor_friction=RIGID_BODY_FRICTION,
        collision_iterations=2,
        positional_correction=0.8,
    )
    world.add_bodies(rigid_bodies)
    world.add_soft_body(
        soft_body,
        gravity=SOFT_BODY_GRAVITY,
        bounds=soft_body_bounds,
        time_step=SOFT_BODY_DT,
    )
    world.add_emitter(particle_system)

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            ik_target[0] -= TARGET_SPEED
        if keys[pygame.K_RIGHT]:
            ik_target[0] += TARGET_SPEED
        if keys[pygame.K_UP]:
            ik_target[1] -= TARGET_SPEED
        if keys[pygame.K_DOWN]:
            ik_target[1] += TARGET_SPEED
        if pygame.mouse.get_pressed()[0]:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            ik_target = [mouse_x, mouse_y]

        time_seconds = pygame.time.get_ticks() / 1000.0
        fk_points = fk_chain.points([time_seconds, time_seconds * 2.0])
        ik_angles = ik_solver.solve(ik_angles, tuple(ik_target))
        ik_points = ik_chain.points(ik_angles)

        world.update(dt)

        screen.fill(BACKGROUND_COLOR)
        particle_system.draw(screen)
        pygame.draw.line(
            screen,
            PANEL_LINE_COLOR,
            (0, SCREEN_HEIGHT * 0.5),
            (SCREEN_WIDTH, SCREEN_HEIGHT * 0.5),
            2,
        )
        pygame.draw.line(
            screen,
            PANEL_LINE_COLOR,
            (SCREEN_WIDTH * 0.5, 0),
            (SCREEN_WIDTH * 0.5, SCREEN_HEIGHT),
            2,
        )
        _draw_forward_kinematics(screen, fk_points)
        _draw_inverse_kinematics(screen, ik_points, ik_target)
        _draw_rigid_bodies(screen, rigid_bodies)
        soft_body.draw(
            screen,
            particle_color=SOFT_BODY_PARTICLE,
            spring_color=SOFT_BODY_SPRING,
            fill_color=SOFT_BODY_FILL,
        )

        pygame.display.flip()

    pygame.quit()


def _draw_forward_kinematics(surface, points: list[tuple[float, float]]) -> None:
    pygame.draw.circle(surface, FK_BASE_COLOR, points[0], BASE_RADIUS)
    pygame.draw.line(surface, FK_LINK_1_COLOR, points[0], points[1], LINK_WIDTH)
    pygame.draw.line(surface, FK_LINK_2_COLOR, points[1], points[2], LINK_WIDTH)


def _draw_inverse_kinematics(
    surface,
    points: list[tuple[float, float]],
    target: list[float],
) -> None:
    for index in range(len(points) - 1):
        pygame.draw.line(surface, IK_LINK_COLOR, points[index], points[index + 1], 5)
        pygame.draw.circle(
            surface,
            IK_JOINT_COLOR,
            (int(points[index][0]), int(points[index][1])),
            8,
        )

    pygame.draw.circle(
        surface,
        IK_END_COLOR,
        (int(points[-1][0]), int(points[-1][1])),
        10,
    )
    pygame.draw.circle(surface, IK_TARGET_COLOR, (int(target[0]), int(target[1])), 6)


def _draw_rigid_bodies(surface, bodies: list[Body]) -> None:
    for index, body in enumerate(bodies):
        if body.inv_mass == 0.0:
            color = RIGID_BODY_STATIC_COLOR
        else:
            color = RIGID_BODY_COLORS[index % len(RIGID_BODY_COLORS)]

        pygame.draw.circle(surface, color, body.position, body.radius)
        pygame.draw.circle(surface, RIGID_BODY_OUTLINE, body.position, body.radius, 2)


if __name__ == "__main__":
    main()
