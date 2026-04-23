import pygame

from engine import Bounds, SoftBody

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60
GRAVITY = (0.0, 700.0)
BACKGROUND_COLOR = (245, 247, 250)
WALL_COLOR = (70, 76, 89)
SOFT_BODY_FILL = (244, 132, 132)
SOFT_BODY_PARTICLE = (180, 36, 36)
SOFT_BODY_SPRING = (45, 55, 72)

def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Soft Body Simulation")
    clock = pygame.time.Clock()

    bounds = Bounds(24, 24, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 24)
    soft_body = SoftBody(
        center=(SCREEN_WIDTH * 0.5, 160),
        particle_count=12,
        radius=58,
        particle_radius=5,
        spring_stiffness=36.0,
        spring_damping=4.0,
        shape_stiffness=24.0,
        shape_damping=5.0,
        pressure=3.5,
        min_area_ratio=0.72,
        restitution=0.72,
    )

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        soft_body.update(dt, gravity=GRAVITY, bounds=bounds)

        screen.fill(BACKGROUND_COLOR)
        _draw_bounds(screen, bounds)
        soft_body.draw(
            screen,
            particle_color=SOFT_BODY_PARTICLE,
            spring_color=SOFT_BODY_SPRING,
            fill_color=SOFT_BODY_FILL,
        )
        pygame.display.flip()

    pygame.quit()


def _draw_bounds(screen, bounds: Bounds) -> None:
    rect = pygame.Rect(
        bounds.min_x,
        bounds.min_y,
        bounds.max_x - bounds.min_x,
        bounds.max_y - bounds.min_y,
    )
    pygame.draw.rect(screen, WALL_COLOR, rect, width=3)


if __name__ == "__main__":
    main()
