import pygame
import math
import random


class Particle:
    def __init__(self, pos, color=(255, 255, 255)):
        self.pos = pygame.Vector2(pos)
        angle = math.radians(random.uniform(0, 360))
        speed = random.uniform(10, 30)
        self.vel = pygame.Vector2(math.cos(angle), math.sin(angle)) * speed
        self.radius = random.randint(2, 4)
        self.color = color
        self.faded_color = tuple(int(c * 0.7) for c in self.color)
        self.lifetime = 0.2
        self.age = 0
        self.width = random.randint(1, 2)

    def update(self, dt):
        self.pos += self.vel * dt
        self.age += dt
        if self.age < self.lifetime:
            fade_factor = 1 - (self.age / self.lifetime)
            self.color = tuple(int(c * fade_factor) for c in self.faded_color)

    def draw(self, surface):
        if self.age < self.lifetime:
            pygame.draw.circle(
                surface,
                self.color,
                (int(self.pos.x), int(self.pos.y)),
                self.radius,
                self.width,
            )

    def is_alive(self):
        return self.age < self.lifetime
