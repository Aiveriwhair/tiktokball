import pygame
import sys
import math
import pygame.gfxdraw

# Constants
WIDTH, HEIGHT = 800, 600
FPS = 200
GRAVITY = 800
BOUNCE_DAMPING = 1  # Reduces speed after a bounce
MASS = 1  # Simplification: treat mass as 1 for now
RADIUS = 20  # Radius of the ball
SIMULATION_CENTER = (WIDTH / 2, HEIGHT / 2)  # Center of the simulation area


# Ball Class (manual physics, without collision calculation)
class Ball:
    def __init__(self, pos, radius=RADIUS):
        self.pos = pygame.Vector2(pos)
        self.radius = radius
        self.vel = pygame.Vector2(5, 0)  # Initial velocity is 0
        self.acc = pygame.Vector2(0, GRAVITY)  # Gravity acts on the ball

    def update(self, dt):
        # Apply gravity
        self.vel += self.acc * dt

        # Update position based on velocity
        self.pos += self.vel * dt

    def draw(self, surface):
        pygame.draw.circle(
            surface, (255, 0, 0), (int(self.pos.x), int(self.pos.y)), self.radius, 40
        )

    def check_collisions(self, obstacles):
        if not obstacles:
            return
        obstacle = obstacles[0]

        to_ball = self.pos - obstacle.pos
        distance = to_ball.length()
        max_distance = obstacle.radius - self.radius
        # Si la balle dépasse le rayon intérieur
        if distance > max_distance:
            # Vérifie si la balle est dans une zone ouverte
            if obstacle.is_open_at(self.pos):
                return
            if distance == 0:
                # Éviter division par zéro si centres superposés
                normal = pygame.Vector2(1, 0)
            else:
                normal = -to_ball.normalize()
            # Corrige position (remet la balle juste au bord intérieur)
            self.pos = obstacle.pos + to_ball.normalize() * max_distance
            # Rebondir proprement (élastique ou avec damping)
            self.vel = self.vel.reflect(normal) * BOUNCE_DAMPING


class CircularObstacle:
    def __init__(
        self,
        pos,
        radius,
        open_angle_range=None,  # (min_angle, max_angle) in degrees
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
        self.open_angle_range = open_angle_range  # in degrees
        self.rotationSpeed = rotationSpeed
        self.shrink_speed = 500

    def is_open_at(self, point):
        if self.open_angle_range is None:
            return False
        direction = point - self.pos
        angle = math.degrees(math.atan2(-direction.y, direction.x)) % 360
        min_angle, max_angle = self.open_angle_range

        if min_angle <= max_angle:
            # Cas normal
            return min_angle <= angle <= max_angle
        else:
            # La plage dépasse 360° et "wrap"
            return angle >= min_angle or angle <= max_angle

    def update(self, dt):
        # Smooth transition to the target radius
        if abs(self.radius - self.target_radius) > 1:
            direction = 1 if self.target_radius > self.radius else -1
            self.radius += direction * self.shrink_speed * dt
            # Clamp pour éviter de dépasser
            if (direction == 1 and self.radius > self.target_radius) or (
                direction == -1 and self.radius < self.target_radius
            ):
                self.radius = self.target_radius

        # Update the open angle range based on rotation speed
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
        # Normaliser les angles
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

        # Convertir en radians et tracer des segments
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


# Simulation Class
class Simulation:
    def __init__(self, gravity=GRAVITY, bounce_damping=BOUNCE_DAMPING):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Manual Physics Ball Inside Multiple Circular Walls")
        self.clock = pygame.time.Clock()
        self.running = True

        # Ball inside the circle
        self.ball = Ball(pos=SIMULATION_CENTER)  # Start inside the first circle

        # List to hold multiple circular obstacles
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

    def update(self, dt):
        for obstacle in self.obstacles:
            obstacle.update(dt)

        self.ball.update(dt)

        self.obstacles = [o for o in self.obstacles if not o.should_destroy(self.ball)]

        # Update the obstacles radius to make them smaller
        print("Updating obstacles")
        for i, obstacle in enumerate(self.obstacles):
            obstacle.target_radius = 200 + i * 20
            print(obstacle.target_radius, obstacle.radius)

        steps = 5
        for _ in range(steps):
            self.ball.check_collisions(self.obstacles)

    def draw(self):
        self.screen.fill((0, 0, 0))  # Fill screen with white background
        for obstacle in self.obstacles:
            obstacle.draw(self.screen)
        self.ball.draw(self.screen)

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


# Example usage:
if __name__ == "__main__":
    sim = Simulation()
    sim.run()
