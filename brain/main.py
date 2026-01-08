# this is for running the simulations, we just import everything from other files to keep it clean.

import pygame
from map import Map

# --- Constants ---
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60

# --- Colors ---
GREEN = (50, 200, 50)  # A nice grassy color for the background


def main():
    """ Main program function. """
    pygame.init()

    screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])
    pygame.display.set_caption("Traffic Simulation")
    clock = pygame.time.Clock()

    # Create the map
    simulation_map = Map()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # --- Drawing ---
        screen.fill(GREEN)  # Draw a background color
        simulation_map.draw(screen)  # Draw the map
        pygame.display.flip()  # Update the display
        clock.tick(FPS)  # Limit frames per second

    pygame.quit()


if __name__ == "__main__":
    main()
