import pygame
import random
import colorsys
from math import cos, sin
from typing import List

from map import Map
from car_logic import Car, CarState
from path_manager import PathManager
from traffic_logic import TrafficLightManager, update_car_states, CAR_BOUNDING_BOX_SIZE

# --- Constants ---
SCREEN_WIDTH = 1600
SCREEN_HEIGHT = 1200
FPS = 60

# --- CONFIGURATION: GRID SETTINGS ---
GRID_COLS = 6
GRID_ROWS = 6
MAX_CARS = 30
ACTIVE_NODES = [
    1, 2, 3, 4, 5, 6,
    7, 9, 10, 12, 13,
    16, 17, 18, 19, 20, 21,
    23, 25, 26, 27, 28, 29, 30,
     32, 33, 34, 35, 36
]

# --- VEHICLE TYPES (NO ACCELERATION) ---
VEHICLE_TYPES = {
    'Standard': {
        'length': 10,
        'max_speed': 69,
        'engine_power': 375,
        'weight': 1500,
        'speed_stop_factor': 0.16,
        'brake_force': 110
    },
    'Sports': {
        'length': 10,
        'max_speed': 100,
        'engine_power': 630,
        'weight': 1400,
        'speed_stop_factor': 0.25,
        'brake_force': 175
    },
    'Truck': {
        'length': 22,
        'max_speed': 36,
        'engine_power': 208,
        'weight': 2600,
        'speed_stop_factor': 0.05,
        'brake_force': 40
    },
    'Van': {
        'length': 15,
        'max_speed': 55,
        'engine_power': 315,
        'weight': 2100,
        'speed_stop_factor': 0.1,
        'brake_force': 75
    }
}

# --- SPAWN RATES ---
VEHICLE_WEIGHTS = {
    'Standard': 20,
    'Sports': 10,
    'Truck': 15,
    'Van': 25
}

# --- Colors ---
BACKGROUND_COLOR = (200, 200, 200)
BOUNDING_BOX_COLOR = {
    CarState.ACCELERATE: (0, 255, 0),
    CarState.CRUISE: (255, 255, 0),
    CarState.BRAKE: (255, 0, 0)
}
TEXT_COLOR = (0, 0, 0)


# --- DYNAMIC COLOR GENERATOR ---
def generate_distinct_colors(num_colors):
    colors = []
    for i in range(num_colors):
        hue = i / num_colors
        rgb_float = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
        rgb_int = (int(rgb_float[0] * 255), int(rgb_float[1] * 255), int(rgb_float[2] * 255))
        colors.append(rgb_int)
    return colors


CAR_COLORS = generate_distinct_colors(MAX_CARS)


class ViewMode:
    DEBUG = 1
    INFO = 2
    CLEAN = 3


pygame.init()
pygame.font.init()
font = pygame.font.SysFont('Arial', 14, bold=True)
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Traffic Simulation (1: Debug, 2: Info, 3: Clean)")
clock = pygame.time.Clock()


def draw_car(car: Car, view_mode, render_scale):
    """Draws the car, scaled dynamically to the map."""

    # Visual dimensions
    scaled_len = car.length * render_scale
    scaled_width = 5 * render_scale  # Half-width

    # --- Draw Car Triangle ---
    shape = [(scaled_len, 0), (-scaled_width, -scaled_width), (-scaled_width, scaled_width)]

    rotated_shape = []
    for x, y in shape:
        rotated_x = x * cos(car.angle) - y * sin(car.angle)
        rotated_y = x * sin(car.angle) + y * cos(car.angle)
        translated_x = rotated_x + car.x
        translated_y = rotated_y + car.y
        rotated_shape.append((translated_x, translated_y))
    pygame.draw.polygon(screen, car.color, rotated_shape)

    # --- Draw Dynamic Bounding Box (Debug Only) ---
    if view_mode == ViewMode.DEBUG:
        corners = [
            (scaled_len, -scaled_width),
            (scaled_len, scaled_width),
            (-scaled_width, scaled_width),
            (-scaled_width, -scaled_width)
        ]

        rotated_corners = []
        for x, y in corners:
            # Rotate
            rx = x * cos(car.angle) - y * sin(car.angle)
            ry = x * sin(car.angle) + y * cos(car.angle)
            # Translate
            rotated_corners.append((car.x + rx, car.y + ry))

        color = BOUNDING_BOX_COLOR.get(car.state, (0, 255, 0))
        pygame.draw.lines(screen, color, True, rotated_corners, 1)

        # Draw Speed
        speed_surf = font.render(f"{int(car.speed)}", True, TEXT_COLOR)
        screen.blit(speed_surf, (car.x - 10, car.y + 10))

    if view_mode in [ViewMode.DEBUG, ViewMode.INFO]:
        if car.node_path and car.node_path_index < len(car.node_path) - 1:
            next_node = car.node_path[car.node_path_index + 1]
            text_surf = font.render(f"Next: {next_node}", True, TEXT_COLOR)
            screen.blit(text_surf, (car.x - 20, car.y - 30))


def get_random_node_path(sim_map: 'Map', exclude_starts=None, min_len=25):
    if exclude_starts is None: exclude_starts = set()
    attempts = 0
    while attempts < 100:
        available_starts = [n for n in sim_map.nodes.keys() if n not in exclude_starts]
        if not available_starts: available_starts = list(sim_map.nodes.keys())

        start_node = random.choice(available_starts)
        path = [start_node]
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
    return path


def main():
    running = True
    current_view_mode = ViewMode.DEBUG

    simulation_map = Map(
        screen_width=SCREEN_WIDTH,
        screen_height=SCREEN_HEIGHT,
        grid_cols=GRID_COLS,
        grid_rows=GRID_ROWS,
        active_nodes=ACTIVE_NODES
    )

    path_manager = PathManager(simulation_map)
    traffic_light_manager = TrafficLightManager(simulation_map)

    cars: List[Car] = []
    occupied_start_nodes = set()

    vehicle_types = list(VEHICLE_WEIGHTS.keys())
    vehicle_weights = list(VEHICLE_WEIGHTS.values())

    for i in range(MAX_CARS):
        car_color = CAR_COLORS[i % len(CAR_COLORS)]

        v_type_name = random.choices(vehicle_types, weights=vehicle_weights, k=1)[0]
        v_props = VEHICLE_TYPES[v_type_name]

        # REMOVED ACCELERATION ARGUMENT
        new_car = Car(
            map_data=simulation_map,
            color=car_color,
            name=f"{v_type_name} {i}",
            max_speed=v_props['max_speed'],
            length=v_props['length'],
            engine_power=v_props['engine_power'],
            weight=v_props['weight'],
            speed_stop_factor=v_props['speed_stop_factor'],
            brake_force=v_props['brake_force']
        )

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
                elif event.key == pygame.K_2:
                    current_view_mode = ViewMode.INFO
                elif event.key == pygame.K_3:
                    current_view_mode = ViewMode.CLEAN

        update_car_states(cars, traffic_light_manager, path_manager)

        for car in cars:
            car.update(dt, path_manager)

        screen.fill(BACKGROUND_COLOR)

        simulation_map.draw(screen, traffic_light_manager, path_manager, current_view_mode)

        for car in cars:
            draw_car(car, current_view_mode, simulation_map.render_scale)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()