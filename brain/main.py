import pygame
import random
from math import cos, sin
from typing import List

from map import Map
from car_logic import Car
from path_manager import PathManager
from traffic_logic import TrafficLightManager, update_car_states, CAR_BOUNDING_BOX_SIZE

# --- Constants ---
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60
CAR_SPEED = 50
MAX_CARS = 6

# --- Colors ---
BACKGROUND_COLOR = (200, 200, 200)
CAR_COLORS = [(255, 0, 0), (0, 0, 255), (0, 255, 0), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
BOUNDING_BOX_COLOR = (0, 255, 0)
TEXT_COLOR = (0, 0, 0)


# --- View Modes ---
class ViewMode:
    DEBUG = 1
    INFO = 2
    CLEAN = 3


# --- Pygame Setup ---
pygame.init()
pygame.font.init()
font = pygame.font.SysFont('Arial', 14, bold=True)
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Traffic Simulation (1: Debug, 2: Info, 3: Clean)")
clock = pygame.time.Clock()


def draw_car(car: Car, view_mode):
    """Draws the car, and optionally debug info."""

    # --- Draw Bounding Box (Debug Only) ---
    if view_mode == ViewMode.DEBUG:
        half_box = CAR_BOUNDING_BOX_SIZE / 2
        pygame.draw.rect(screen, BOUNDING_BOX_COLOR,
                         (car.x - half_box, car.y - half_box, CAR_BOUNDING_BOX_SIZE, CAR_BOUNDING_BOX_SIZE), 1)

    # --- Draw Car Triangle ---
    shape = [(10, 0), (-5, -5), (-5, 5)]
    rotated_shape = []
    for x, y in shape:
        rotated_x = x * cos(car.angle) - y * sin(car.angle)
        rotated_y = x * sin(car.angle) + y * cos(car.angle)
        translated_x = rotated_x + car.x
        translated_y = rotated_y + car.y
        rotated_shape.append((translated_x, translated_y))
    pygame.draw.polygon(screen, car.color, rotated_shape)

    # --- Draw Next Target Node (Debug & Info) ---
    if view_mode in [ViewMode.DEBUG, ViewMode.INFO]:
        # Check if we have a path and are not at the absolute end
        if car.node_path and car.node_path_index < len(car.node_path) - 1:
            # Display the IMMEDIATE next node, not the final destination
            next_node = car.node_path[car.node_path_index + 1]
            text_surf = font.render(f"Next: {next_node}", True, TEXT_COLOR)
            screen.blit(text_surf, (car.x - 20, car.y - 30))


def get_random_node_path(sim_map: 'Map', exclude_starts=None, min_len=25):
    """Generates a random valid path, optionally avoiding specific start nodes."""
    if exclude_starts is None:
        exclude_starts = set()

    attempts = 0
    while attempts < 100:
        # Pick a start node that isn't in the excluded set
        available_starts = [n for n in sim_map.nodes.keys() if n not in exclude_starts]
        if not available_starts:
            available_starts = list(sim_map.nodes.keys())  # Fallback if full

        start_node = random.choice(available_starts)
        path = [start_node]

        # Build path
        valid_path = True
        for _ in range(min_len + random.randint(0, 5)):
            try:
                prev_node = path[-2] if len(path) > 1 else None
                neighbors = [n for n in sim_map.graph.get_neighbors(path[-1], prev_node)]
                if not neighbors:
                    valid_path = False
                    break
                path.append(random.choice(neighbors))
            except (IndexError, KeyError):
                valid_path = False
                break

        if valid_path and len(path) >= min_len:
            return path
        attempts += 1
    return path  # Return whatever we got if we timed out


def main():
    """Main simulation loop."""
    running = True
    current_view_mode = ViewMode.DEBUG

    simulation_map = Map()
    path_manager = PathManager(simulation_map)
    traffic_light_manager = TrafficLightManager(simulation_map)

    cars: List[Car] = []
    occupied_start_nodes = set()

    for i in range(MAX_CARS):
        car_color = CAR_COLORS[i % len(CAR_COLORS)]
        new_car = Car(speed=CAR_SPEED, map_data=simulation_map, color=car_color, name=f"Car {i}")

        # Ensure unique start nodes to prevent 'Deadlock on Spawn'
        node_path = get_random_node_path(simulation_map, exclude_starts=occupied_start_nodes)

        if node_path:
            occupied_start_nodes.add(node_path[0])
            new_car.set_path(node_path, path_manager)
            cars.append(new_car)

    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    current_view_mode = ViewMode.DEBUG
                    pygame.display.set_caption("Traffic Simulation - DEBUG MODE")
                elif event.key == pygame.K_2:
                    current_view_mode = ViewMode.INFO
                    pygame.display.set_caption("Traffic Simulation - INFO MODE")
                elif event.key == pygame.K_3:
                    current_view_mode = ViewMode.CLEAN
                    pygame.display.set_caption("Traffic Simulation - CLEAN MODE")

        update_car_states(cars, traffic_light_manager, path_manager)

        for car in cars:
            car.update(dt, path_manager)

        screen.fill(BACKGROUND_COLOR)
        simulation_map.draw(screen, traffic_light_manager, path_manager, current_view_mode)

        for car in cars:
            draw_car(car, current_view_mode)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()