import pygame
import math
import random
from map import Map
from car_logic import Car, a_star_search, CarState
import traffic_logic

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
    """Draws the car as a triangle pointing in its direction of travel.

    If the car is in `WAITING` state, a small red circle with pause bars is
    drawn above the car to indicate it is stopped waiting for an obstacle.
    """
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

    # Visual indicator for WAITING state: small red circle with pause bars above the car
    try:
        if hasattr(car, 'state') and car.state == CarState.WAITING:
            cx = int(car.x)
            cy = int(car.y - 18)  # place above the car
            radius = 8
            # Outer circle (red)
            pygame.draw.circle(screen, RED, (cx, cy), radius)
            # Pause bars (two small white rectangles)
            bar_w = 2
            bar_h = 8
            gap = 3
            left_rect = (cx - gap - bar_w, cy - bar_h // 2, bar_w, bar_h)
            right_rect = (cx + gap, cy - bar_h // 2, bar_w, bar_h)
            pygame.draw.rect(screen, WHITE, left_rect)
            pygame.draw.rect(screen, WHITE, right_rect)
            # Thin black outline for visibility
            pygame.draw.circle(screen, (0,0,0), (cx, cy), radius, 1)
    except Exception:
        # Be defensive: drawing should never crash the main loop
        pass

    # Draw detection cone in front of moving cars for visualization
    try:
        if getattr(car, 'speed', 0) > 0:
            draw_detection_cone(screen, car, (255, 0, 0))
    except Exception:
        pass


def draw_detection_cone(screen, car, color=(255,0,0)):
    """Draw a translucent, blinking trapezium in front of the car to visualize detection."""
    try:
        import traffic_logic as tl
        # Build trapezium points: near base (center at car + heading*NEAR_BASE_DIST)
        heading = pygame.math.Vector2(math.cos(car.angle), math.sin(car.angle))
        perp = heading.rotate(-90)

        near_c = pygame.math.Vector2(car.x, car.y) + heading * tl.NEAR_BASE_DIST
        far_c = near_c + heading * tl.DETECTION_HEIGHT

        near_half = tl.NEAR_BASE_WIDTH / 2.0
        far_half = tl.FAR_BASE_WIDTH / 2.0

        near_left = near_c + perp * near_half
        near_right = near_c - perp * near_half
        far_left = far_c + perp * far_half
        far_right = far_c - perp * far_half

        # Blinking alpha via sinusoidal function (1 Hz pulse)
        t = pygame.time.get_ticks() / 1000.0
        base_alpha = 80
        alpha = int((0.5 + 0.5 * math.sin(2 * math.pi * 1.0 * t)) * base_alpha)
        alpha = max(15, alpha)

        surf = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        pygame.draw.polygon(surf, (color[0], color[1], color[2], alpha), [near_left, far_left, far_right, near_right])
        screen.blit(surf, (0, 0))
    except Exception:
        pass

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
            num_stops = random.randint(6, 9)
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


def get_car_positions(cars):
    """Return current positions for cars as {name: (x, y, angle)}.

    If `cars` is None or empty, returns an empty dict.
    """
    if not cars:
        return {}
    return {c.name: (c.x, c.y, c.angle) for c in cars}

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

    # Initialize 6 cars
    cars = initialise_cars(6, simulation_map, stops_list=stops_list)

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0 # Get delta time in seconds
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update obstacle/waiting states based on traffic logic (stop cars when blocked)
        traffic_logic.update_car_waiting_states(simulation_map, cars)

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