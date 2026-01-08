import pygame
import math
from map import Map
from car_logic import Car, a_star_search

# --- Constants ---
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60

# --- Colors ---
GREEN = (50, 200, 50)
RED = (255, 0, 0)
WHITE = (255, 255, 255)

def create_multi_stop_path(stops, graph):
    """Creates a single path from a list of sequential stops, preventing 180-degree turns."""
    if not stops or len(stops) < 2:
        return []

    full_path = []
    previous_node = None

    for i in range(len(stops) - 1):
        start_node = stops[i]
        end_node = stops[i+1]
        
        # When creating a path segment, tell the search where it came from.
        segment_path = a_star_search(start_node, end_node, graph, initial_previous_node=previous_node)
        
        if not segment_path:
            print(f"Warning: No path found from {start_node} to {end_node}.")
            # If a segment fails, we can't reliably continue the path.
            return full_path 

        # The previous node for the *next* segment is the second-to-last node of this one.
        if len(segment_path) > 1:
            previous_node = segment_path[-2]
        else:
            # This case handles direct connections where the segment is just [start, end].
            previous_node = start_node

        # For all segments after the first, remove the first node to avoid duplication.
        if i > 0:
            segment_path.pop(0)
            
        full_path.extend(segment_path)
        
    return full_path

def draw_car(screen, car):
    """Draws the car as a triangle pointing in its direction of travel."""
    car_length = 20
    car_width = 10
    
    p1 = (
        car.x + car_length / 2 * math.cos(car.angle),
        car.y + car_length / 2 * math.sin(car.angle)
    )
    p2 = (
        car.x - car_length / 2 * math.cos(car.angle) + car_width / 2 * math.sin(car.angle),
        car.y - car_length / 2 * math.sin(car.angle) - car_width / 2 * math.cos(car.angle)
    )
    p3 = (
        car.x - car_length / 2 * math.cos(car.angle) - car_width / 2 * math.sin(car.angle),
        car.y - car_length / 2 * math.sin(car.angle) + car_width / 2 * math.cos(car.angle)
    )
    
    pygame.draw.polygon(screen, RED, [p1, p2, p3])

def main():
    """ Main program function. """
    pygame.init()

    screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])
    pygame.display.set_caption("Traffic Simulation")
    clock = pygame.time.Clock()

    simulation_map = Map()
    graph = simulation_map.graph

    # Define the sequence of stops for the car using the new number system
    stops = [1, 9, 3, 4, 5, 1, 7, 2]
    node_path = create_multi_stop_path(stops, graph)

    # Initialize the car with the precise state-machine logic
    car = Car(speed=100, map_data=simulation_map)
    car.set_path(node_path)

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0 # Get delta time in seconds
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Pass delta time to the update function
        car.update(dt)

        screen.fill(GREEN)
        simulation_map.draw(screen)
        draw_car(screen, car)
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
