import pygame
import sys
import math
import pygame.gfxdraw
import random

from particle import Particle

WIDTH, HEIGHT = 800, 600  # Window dimensions
FPS = 200  # Frames per second
GRAVITY = 800  # Acceleration due to gravity
BOUNCE_DAMPING = 1  # Reduces speed after a bounce
MASS = 1  # Mass of the ball
SIMULATION_CENTER = (WIDTH / 2, HEIGHT / 2)  # Center of the simulation area
BALL_RADIUS = 20  # Radius of the ball


class Ball:
    def __init__(self, pos, radius):
        self.pos = pygame.Vector2(pos)
        self.radius = radius
        self.vel = pygame.Vector2(5, 0)
        self.acc = pygame.Vector2(0, GRAVITY)
        self.trail = []
        self.max_trail_length = 20

    def update(self, dt):
        self.vel += self.acc * dt
        self.pos += self.vel * dt

        self.trail.append(self.pos.copy())
        if len(self.trail) > self.max_trail_length:
            self.trail.pop(0)

    def draw(self, surface):
        for i, pos in enumerate(self.trail):
            alpha = int(255 * (i / len(self.trail)))  # Opacité
            radius = self.radius  # Toujours la même taille que la balle
            trail_surface = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
            pygame.draw.circle(
                trail_surface, (255, 0, 0, alpha), (radius, radius), radius
            )
            surface.blit(trail_surface, (pos.x - radius, pos.y - radius))

        # Dessine la balle noire (toujours au-dessus)
        pygame.draw.circle(
            surface, (0, 0, 0), (int(self.pos.x), int(self.pos.y)), self.radius
        )
        pygame.draw.circle(  # Contour rouge
            surface, (255, 0, 0), (int(self.pos.x), int(self.pos.y)), self.radius, 2
        )

    def check_collisions(self, obstacles):
        if not obstacles:
            return
        obstacle = obstacles[0]

        to_ball = self.pos - obstacle.pos
        distance = to_ball.length()
        max_distance = obstacle.radius - self.radius
        if distance > max_distance:
            if obstacle.is_open_at(self.pos):
                return
            if distance == 0:
                normal = pygame.Vector2(1, 0)
            else:
                normal = -to_ball.normalize()
            self.pos = obstacle.pos + to_ball.normalize() * max_distance
            self.vel = self.vel.reflect(normal) * BOUNCE_DAMPING


class CircularObstacle:
    def __init__(
        self,
        pos,
        radius,
        open_angle_range=None,
        color=(150, 150, 150),
        elasticity=0.8,
        friction=0.7,
        rotationSpeed=0,
    ):
        self.pos = pygame.Vector2(pos)
        self.radius = radius
        self.target_radius = radius
        self.color = color
        self.elasticity = elasticity
        self.friction = friction
        self.open_angle_range = open_angle_range
        self.rotationSpeed = rotationSpeed
        self.shrink_speed = 500

    def is_open_at(self, point):
        if self.open_angle_range is None:
            return False
        direction = point - self.pos
        angle = math.degrees(math.atan2(-direction.y, direction.x)) % 360
        min_angle, max_angle = self.open_angle_range

        if min_angle <= max_angle:
            return min_angle <= angle <= max_angle
        else:
            return angle >= min_angle or angle <= max_angle

    def update(self, dt):

        if abs(self.radius - self.target_radius) > 1:
            direction = 1 if self.target_radius > self.radius else -1
            self.radius += direction * self.shrink_speed * dt

            if (direction == 1 and self.radius > self.target_radius) or (
                direction == -1 and self.radius < self.target_radius
            ):
                self.radius = self.target_radius

        if self.open_angle_range is not None:
            min_angle, max_angle = self.open_angle_range
            min_angle += self.rotationSpeed * dt
            max_angle += self.rotationSpeed * dt
            self.open_angle_range = (min_angle % 360, max_angle % 360)

    def draw(self, surface):
        if self.open_angle_range is None:
            pygame.draw.circle(
                surface,
                self.color,
                (int(self.pos.x), int(self.pos.y)),
                self.radius,
                width=5,
            )
        else:
            min_angle, max_angle = self.open_angle_range
            self.dessine_arc(surface, min_angle, max_angle)

    def draw_open_circle(self, surface, min_angle, max_angle, step=1, width=4):
        min_angle %= 360
        max_angle %= 360

        angles = []
        for angle in range(0, 360, step):
            if min_angle < max_angle:
                if not (min_angle <= angle <= max_angle):
                    angles.append(angle)
            else:
                if not (angle >= min_angle or angle <= max_angle):
                    angles.append(angle)

        points = []
        for angle in angles:
            rad = math.radians(angle)
            x = int(self.pos.x + math.cos(rad) * self.radius)
            y = int(self.pos.y - math.sin(rad) * self.radius)
            points.append((x, y))

        for i in range(len(points) - 1):
            pygame.draw.line(surface, self.color, points[i], points[i + 1], width)

    def dessine_arc(
        self,
        surface,
        start_angle,
        stop_angle,
        color=(255, 255, 255),
    ):
        x, y = self.pos
        start_angle = int(start_angle % 360)
        stop_angle = int(stop_angle % 360)
        if start_angle == stop_angle:
            pygame.gfxdraw.circle(surface, x, y, self.radius, color)
        else:
            pygame.gfxdraw.arc(
                surface,
                int(x),
                int(y),
                int(self.radius),
                int(-start_angle),
                int(-stop_angle),
                color,
            )

    def should_destroy(self, ball):
        to_ball = ball.pos - self.pos
        distance = to_ball.length()

        is_escaping = distance > self.radius + ball.radius * 0.5
        return is_escaping and self.is_open_at(ball.pos)


class Simulation:
    def __init__(self, gravity=GRAVITY, bounce_damping=BOUNCE_DAMPING):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("TikTok Ball")
        self.clock = pygame.time.Clock()
        self.running = True
        self.particles = []

        self.ball = Ball(pos=SIMULATION_CENTER, radius=BALL_RADIUS)

        self.obstacles = []
        for i in range(100):
            self.obstacles.append(
                CircularObstacle(
                    pos=SIMULATION_CENTER,
                    radius=200 + i * 20,
                    open_angle_range=((60 + i * 4) % 360, (120 + i * 4) % 360),
                    rotationSpeed=30,
                )
            )

        self.gravity = gravity
        self.bounce_damping = bounce_damping

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

    def spawn_particles(self, pos, color=(255, 255, 255), count=30, radius=30):
        for _ in range(count):
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(radius - 5, radius + 5)  # épaisseur du cercle
            offset = pygame.Vector2(math.cos(angle), math.sin(angle)) * distance
            particle_pos = pos + offset
            particle = Particle(particle_pos, color=color)
            particle.vel = offset.normalize() * random.uniform(10, 30)
            self.particles.append(particle)

    def update(self, dt):
        for obstacle in self.obstacles:
            obstacle.update(dt)

        self.ball.update(dt)

        new_obstacles = []
        for o in self.obstacles:
            if o.should_destroy(self.ball):
                self.spawn_particles(o.pos, color=o.color, radius=o.radius)
            else:
                new_obstacles.append(o)
        self.obstacles = new_obstacles

        for i, obstacle in enumerate(self.obstacles):
            obstacle.target_radius = 200 + i * 20

        steps = 5
        for _ in range(steps):
            self.ball.check_collisions(self.obstacles)

        for p in self.particles:
            p.update(dt)
        self.particles = [p for p in self.particles if p.is_alive()]

    def draw(self):
        self.screen.fill((0, 0, 0))
        for obstacle in self.obstacles[:5]:
            obstacle.draw(self.screen)
        self.ball.draw(self.screen)
        for p in self.particles:
            p.draw(self.screen)

        pygame.display.flip()

    def run(self):
        while self.running:
            dt = self.clock.tick(FPS) / 1000
            self.handle_events()
            self.update(dt)
            self.draw()
            self.clock.tick(FPS)

        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    sim = Simulation()
    sim.run()
