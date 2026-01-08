# kaile to keenan: this is for the car itself, it directly interfaces with the car on how it should move
# we keep the "physical movement" logic of the car in a separate file from its logic and also the larger overall brain

import pygame

class Car(pygame.sprite.Sprite):
    """
    Represents a single car in the simulation.
    This class handles the car's physical properties, movement, and drawing.
    It knows *how* to perform actions, but the 'brain' will decide *when*.
    """
    def __init__(self, x, y, angle=0.0, length=40, width=20, max_speed=5, max_acceleration=0.2):
        super().__init__()
        
        # Create a surface for the car and draw a rectangle on it
        self.image = pygame.Surface((length, width), pygame.SRCALPHA)
        pygame.draw.rect(self.image, (255, 0, 0), (0, 0, length, width))
        self.original_image = self.image
        self.rect = self.image.get_rect(center=(x, y))

        # Movement and position vectors
        self.position = pygame.math.Vector2(x, y)
        self.velocity = pygame.math.Vector2(0.0, 0.0)
        self.angle = angle
        
        # Physics properties
        self.speed = 0.0
        self.acceleration = 0.0
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration

    def accelerate(self):
        """Increases the car's acceleration."""
        self.acceleration = self.max_acceleration

    def brake(self):
        """Applies the brakes, causing deceleration."""
        # Simple brake for now, can be made more realistic
        self.acceleration = -self.max_acceleration * 2

    def update(self):
        """Updates the car's state each frame."""
        # Update speed based on acceleration
        self.speed += self.acceleration
        self.speed = max(0, min(self.speed, self.max_speed)) # Clamp speed

        # Update position based on speed and angle
        self.position.x += self.speed * pygame.math.Vector2(1, 0).rotate(-self.angle).x
        self.position.y += self.speed * pygame.math.Vector2(1, 0).rotate(-self.angle).y
        
        # Update the rect for drawing
        self.image = pygame.transform.rotate(self.original_image, self.angle)
        self.rect = self.image.get_rect(center=self.position)
        
        # Reset acceleration each frame
        self.acceleration = 0