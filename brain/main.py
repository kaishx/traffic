import pygame
import math
import random
from map import Map
from car_logic import Car, a_star_search

# --- Constants ---
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60

# --- Colors ---
GREEN = (50, 200, 50)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
COLORS = [RED, BLUE, (255, 165, 0), (255, 192, 203), (128, 0, 128), (0, 255, 255)]

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

def draw_car(screen, car, color):
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
    
    pygame.draw.polygon(screen, color, [p1, p2, p3])

def initialise_cars(n, simulation_map, stops_list=None):
    """
    Initializes N cars with optional stop lists.
    
    Args:
        n: Number of cars to create
        simulation_map: Map object containing the graph and nodes
        stops_list: Optional list of stop lists for each car. 
                   If None, random stops are generated for each car.
    
    Returns:
        List of initialized Car objects
    """
    graph = simulation_map.graph
    cars = []
    
    # Generate stops if not provided
    if stops_list is None:
        stops_list = []
        all_nodes = list(simulation_map.nodes.keys())
        
        for i in range(n):
            # Generate random stops for this car (3-6 stops)
            num_stops = random.randint(3, 6)
            stops = random.sample(all_nodes, num_stops)
            stops_list.append(stops)
    
    # Create cars
    for i in range(n):
        stops = stops_list[i] if i < len(stops_list) else None
        
        # If stops is None, generate random stops
        if stops is None:
            all_nodes = list(simulation_map.nodes.keys())
            num_stops = random.randint(3, 6)
            stops = random.sample(all_nodes, num_stops)
        
        node_path = create_multi_stop_path(stops, graph)
        
        # Alternate speed between 80-120 units/sec
        speed = 80 + (i % 3) * 20
        color = COLORS[i % len(COLORS)]
        
        car = Car(speed=speed, map_data=simulation_map, name=f"Car{i+1}")
        car.set_path(node_path)
        car.color = color  # Store color for drawing
        
        cars.append(car)
    
    return cars

def main():
    """ Main program function. """
    pygame.init()

    screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])
    pygame.display.set_caption("Traffic Simulation")
    clock = pygame.time.Clock()

    simulation_map = Map()

    # Define stops for the first 2 cars, third car will be auto-generated
    stops_list = [
        [1, 9, 3, 4, 5, 1, 7, 2],
        [2, 4, 8, 6, 3, 7, 9, 5],
        None  # Auto-generate for car 3
    ]

    # Initialize 3 cars
    cars = initialise_cars(3, simulation_map, stops_list=stops_list)

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0 # Get delta time in seconds
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update all cars
        for car in cars:
            car.update(dt)

        screen.fill(GREEN)
        simulation_map.draw(screen)
        
        # Draw all cars
        for car in cars:
            draw_car(screen, car, car.color)
        
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()