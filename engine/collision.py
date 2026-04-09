import pygame

WIDTH, HEIGHT = 800, 600
GRAVITY = 750

FRICTION = 0
COR = 1  # restitution

# FRICTION = 20
# COR = 0.7  # restitution

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Bouncing Balls")
clock = pygame.time.Clock()


class Ball:
    def __init__(self, pos, vel, radius=20):
        self.pos = pygame.math.Vector2(pos)
        self.vel = pygame.math.Vector2(vel)
        self.radius = radius
        self.mass = radius

    def update(self, dt):
        # gravity
        self.vel.y += GRAVITY * dt

        # integrate
        self.pos += self.vel * dt

        # walls
        if self.pos.x <= self.radius:
            self.pos.x = self.radius
            self.vel.x *= -COR

        elif self.pos.x >= WIDTH - self.radius:
            self.pos.x = WIDTH - self.radius
            self.vel.x *= -COR

        if self.pos.y <= self.radius:
            self.pos.y = self.radius
            self.vel.y *= -COR

        elif self.pos.y >= HEIGHT - self.radius:
            self.pos.y = HEIGHT - self.radius
            self.vel.y *= -COR

            # floor friction
            if abs(self.vel.x) > 0:
                friction_force = FRICTION * dt
                self.vel.x -= friction_force * (1 if self.vel.x > 0 else -1)

                if abs(self.vel.x) < 0.01:
                    self.vel.x = 0

    def draw(self, surface):
        pygame.draw.circle(
            surface,
            pygame.Color("red"),
            self.pos,
            self.radius
        )


# COLLISION
def resolve_ball_collision(b1, b2):
    delta = b2.pos - b1.pos
    dist = delta.length()
    min_dist = b1.radius + b2.radius

    if dist == 0 or dist >= min_dist:
        return

    # normal and tangent 
    normal = delta.normalize()
    tangent = pygame.math.Vector2(-normal.y, normal.x)

    # velocities
    v1n = normal.dot(b1.vel)
    v1t = tangent.dot(b1.vel)
    v2n = normal.dot(b2.vel)
    v2t = tangent.dot(b2.vel)

    # 1D collision
    m1, m2 = b1.mass, b2.mass

    v1n_new = (v1n * (m1 - m2) + 2 * m2 * v2n) / (m1 + m2)
    v2n_new = (v2n * (m2 - m1) + 2 * m1 * v1n) / (m1 + m2)

    # restitution
    v1n_new *= COR
    v2n_new *= COR

    # recombine vectors
    b1.vel = normal * v1n_new + tangent * v1t
    b2.vel = normal * v2n_new + tangent * v2t

    # position correction
    penetration = min_dist - dist
    correction = normal * (penetration / 2)
    b1.pos -= correction
    b2.pos += correction


balls = [
    Ball((400, 200), (-300, -200), 20),
    Ball((460, 200), (200, -100), 40),
    Ball((520, 200), (-150, 0), 60),
]


running = True
while running:
    dt = clock.tick(60) / 1000

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # update balls
    for ball in balls:
        ball.update(dt)

    # handle collisions
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            resolve_ball_collision(balls[i], balls[j])

    screen.fill(pygame.Color("black"))
    for ball in balls:
        ball.draw(screen)

    pygame.display.flip()

pygame.quit()