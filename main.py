import pygame

from engine import Bounds, CCDInverseKinematicsSolver, ForwardKinematicsChain, SoftBody

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
BALL_BOUNCINESS = 0.9
BALL_PRESSURE = 0.1
BALL_SPRING_STIFFNESS = 0.1
BALL_SPRING_DAMPING = 0.3


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Soft Body + Forward/Inverse Kinematics")
    clock = pygame.time.Clock()

    fk_base = (SCREEN_WIDTH * 0.25, SCREEN_HEIGHT * 0.25)
    fk_chain = ForwardKinematicsChain(base=fk_base, link_lengths=FK_LINK_LENGTHS)
    ik_base = (SCREEN_WIDTH * 0.68, SCREEN_HEIGHT * 0.25)
    ik_chain = ForwardKinematicsChain(base=ik_base, link_lengths=IK_LINK_LENGTHS)
    ik_solver = CCDInverseKinematicsSolver(ik_chain, iterations=10)
    ik_angles = [0.5, 0.5]
    ik_target = [SCREEN_WIDTH * 0.78, SCREEN_HEIGHT * 0.25]

    soft_body_bounds = Bounds(0, SCREEN_HEIGHT * 0.5, SCREEN_WIDTH, SCREEN_HEIGHT)
    soft_body = SoftBody(
        center=(SCREEN_WIDTH * 0.5, SCREEN_HEIGHT * 0.72),
        particle_count=8,
        radius=50,
        particle_radius=4,
        spring_stiffness=BALL_SPRING_STIFFNESS,
        spring_damping=BALL_SPRING_DAMPING,
        pressure=BALL_PRESSURE,
        restitution=BALL_BOUNCINESS,
    )

    running = True
    while running:
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

        soft_body.update(SOFT_BODY_DT, gravity=SOFT_BODY_GRAVITY, bounds=soft_body_bounds)

        screen.fill(BACKGROUND_COLOR)
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
            (SCREEN_WIDTH * 0.5, SCREEN_HEIGHT * 0.5),
            2,
        )
        _draw_forward_kinematics(screen, fk_points)
        _draw_inverse_kinematics(screen, ik_points, ik_target)
        soft_body.draw(
            screen,
            particle_color=SOFT_BODY_PARTICLE,
            spring_color=SOFT_BODY_SPRING,
            fill_color=SOFT_BODY_FILL,
        )

        pygame.display.flip()
        clock.tick(FPS)

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


if __name__ == "__main__":
    main()
