import pygame
import time
from collections import namedtuple
from math import sqrt, cos, sin
from car_logic import CarState

StopObstacle = namedtuple('StopObstacle', ['x', 'y', 'path_key', 'state'])

# --- CONFIGURATION ---
TRAFFIC_LIGHT_GO_TIME = 10.0
TRAFFIC_LIGHT_YELLOW_TIME = 6.0
TRAFFIC_LIGHT_COOLDOWN_TIME = 1.0

# --- DISTANCES (SIMPLE) ---
# Since we stop INSTANTLY, we don't need huge buffers.
MIN_GAP = 15  # Pixel gap between cars
STOP_LINE_DIST = 10  # Distance to stop before a light

# --- VISUAL CONFIGURATION ---
CAR_BOUNDING_BOX_SIZE = 20  # Required by main.py
CAR_REAR_EXTENT = 5


class JunctionManager:
    def __init__(self, index, sim_map):
        self.index = index
        self.neighbours = []

        nbrs = sim_map.graph.get_neighbors(index)
        if len(nbrs) >= 3:
            self.neighbours = nbrs
        
        self.light_states = {nbr: 'RED' for nbr in self.neighbours}
        self.current_green_index = 0
        self.phase = 'GO'
        self.last_switch_time = time.time()

        if self.neighbours:
            self.light_states[self.neighbours[0]] = 'GREEN'
            self.current_green_index = 0

    def update(self):
        if not self.neighbours: return

        current_time = time.time()
        elapsed = current_time - self.last_switch_time

        if self.phase == 'GO':
            if elapsed > TRAFFIC_LIGHT_GO_TIME:
                self.phase = 'YELLOW'
                self.last_switch_time = current_time
                current_idx = self.current_green_index
                self.light_states[self.neighbours[current_idx]] = 'YELLOW'

        elif self.phase == 'YELLOW':
            if elapsed > TRAFFIC_LIGHT_YELLOW_TIME:
                self.phase = 'COOLDOWN'
                self.last_switch_time = current_time
                current_idx = self.current_green_index
                self.light_states[self.neighbours[current_idx]] = 'RED'

        elif self.phase == 'COOLDOWN':
            if elapsed > TRAFFIC_LIGHT_COOLDOWN_TIME:
                self.phase = 'GO'
                self.last_switch_time = current_time
                next_idx = (self.current_green_index + 1) % len(self.neighbours)
                self.current_green_index = next_idx
                self.light_states[self.neighbours[next_idx]] = 'GREEN'

    def get_time_remaining(self):
        elapsed = time.time() - self.last_switch_time
        if self.phase == 'GO':
            return max(0, TRAFFIC_LIGHT_GO_TIME - elapsed)
        elif self.phase == 'YELLOW':
            return max(0, TRAFFIC_LIGHT_YELLOW_TIME - elapsed)
        else:
            return max(0, TRAFFIC_LIGHT_COOLDOWN_TIME - elapsed)


class TrafficLightManager:
    def __init__(self, sim_map):
        self.sim_map = sim_map
        self.junctions = [node for node, nbrs in sim_map.graph.adjacency_list.items() if len(nbrs) >= 3]

        self.junction_managers = {}

        for node in self.junctions:
            self.junction_managers[node] = JunctionManager(node, sim_map)

    def update(self):
        for jm in self.junction_managers.values():
            jm.update()


def update_car_states(cars, traffic_light_manager, path_manager):
    traffic_light_manager.update()
    scale = traffic_light_manager.sim_map.render_scale

    # Scaled thresholds
    scaled_min_gap = MIN_GAP * scale
    scaled_stop_line = STOP_LINE_DIST * scale

    # Collect GLOBAL obstacles map wide
    stop_obstacles = []
    for j_node in traffic_light_manager.junctions:
        jm = traffic_light_manager.junction_managers[j_node]
        for in_node, state in jm.light_states.items():
            path_key = (in_node, j_node)
            path = path_manager.get_path_for_nodes(path_key)
            if path:
                stop_obstacles.append(StopObstacle(path[-1].x, path[-1].y, path_key, state))

    # Obstacles -> GLOBAL JUNCTION LOCK (Prevent Gridlock)
    occupied_junctions = set()
    for c in cars:
        if c.current_node_sequence and len(c.current_node_sequence) == 3:
            occupied_junctions.add(c.current_node_sequence[1])

    # --- 3. CHECK CONDITIONS ---
    for i, car in enumerate(cars):
        car_path_key = car.current_node_sequence
        if not car_path_key: continue

        is_turning = len(car_path_key) == 3
        # Calculate physics-based distances
        cruise_dist = car.calc_cruise_distance(0.15 * car.speed)
        brake_dist = car.calc_brake_stop_distance()
        
        my_front_extent = car.length * scale

        heading = pygame.math.Vector2(cos(car.angle), sin(car.angle))
        right_normal = pygame.math.Vector2(-sin(car.angle), cos(car.angle))
        next_obstacle = None

        # 1. Check Traffic Lights
        for stop in stop_obstacles:
            if stop.path_key == car_path_key:
                if stop.state == 'GREEN' and stop.path_key[1] not in occupied_junctions:
                    continue
                next_obstacle = stop
                break

        # 2. Check Cars Ahead (Treat as Red Light)
        for other in cars:
            if other is car: continue
            if other.current_node_sequence == car_path_key:
                to_other = pygame.math.Vector2(other.x - car.x, other.y - car.y)
                
                # Is it ahead of me?
                if heading.dot(to_other) > 0:
                    # Calculate position of other car's rear bumper
                    other_rear_dist = (other.length * scale) / 2
                    other_rear_x = other.x - cos(other.angle) * other_rear_dist
                    other_rear_y = other.y - sin(other.angle) * other_rear_dist
                    
                    # Distance to this potential obstacle
                    dist_to_other_sq = (other_rear_x - car.x)**2 + (other_rear_y - car.y)**2
        
                    # If this car is closer than the traffic light (or no light), it's the priority
                    if not next_obstacle or dist_to_other_sq < (next_obstacle.x - car.x)**2 + (next_obstacle.y - car.y)**2:
                        next_obstacle = StopObstacle(other_rear_x, other_rear_y, car_path_key, 'RED')

        if not next_obstacle:
            car.state = CarState.ACCELERATE
            continue

        dist_vec = pygame.math.Vector2(next_obstacle.x - car.x, next_obstacle.y - car.y)
        dist_to_obstacle = dist_vec.length() - my_front_extent

        if next_obstacle.state == 'YELLOW':
            if brake_dist > (dist_to_obstacle - scaled_stop_line):
                car.state = CarState.ACCELERATE
                continue

        respect_dist = dist_to_obstacle - scaled_stop_line

        if respect_dist <= brake_dist:
            car.state = CarState.BRAKE
        elif respect_dist > cruise_dist:
            car.state = CarState.ACCELERATE
        else:
            car.state = CarState.CRUISE