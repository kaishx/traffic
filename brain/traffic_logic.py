import pygame
import time
from collections import namedtuple
from math import sqrt, cos, sin

StopObstacle = namedtuple('StopObstacle', ['x', 'y', 'path_key', 'state'])

# --- CONFIGURATION ---
TRAFFIC_LIGHT_GO_TIME = 8.0
TRAFFIC_LIGHT_YELLOW_TIME = 2.0
TRAFFIC_LIGHT_COOLDOWN_TIME = 1.0

# --- DISTANCES (SIMPLE) ---
# Since we stop INSTANTLY, we don't need huge buffers.
MIN_GAP = 15  # Pixel gap between cars
STOP_LINE_DIST = 10  # Distance to stop before a light

# --- VISUAL CONFIGURATION ---
CAR_BOUNDING_BOX_SIZE = 20  # Required by main.py
CAR_REAR_EXTENT = 5


class TrafficLightManager:
    def __init__(self, sim_map):
        self.sim_map = sim_map
        self.junctions = [node for node, nbrs in sim_map.graph.adjacency_list.items() if len(nbrs) >= 3]

        self.light_states = {}
        self.last_switch_time = {}
        self.current_green_index = {}
        self.junction_phases = {}

        for node in self.junctions:
            self.last_switch_time[node] = time.time()
            self.junction_phases[node] = 'GO'

            neighbors = self.sim_map.graph.get_neighbors(node)
            self.light_states[node] = {nbr: 'RED' for nbr in neighbors}

            if neighbors:
                self.light_states[node][neighbors[0]] = 'GREEN'
                self.current_green_index[node] = 0

    def update(self):
        current_time = time.time()
        for node in self.junctions:
            neighbors = self.sim_map.graph.get_neighbors(node)
            if not neighbors: continue

            elapsed = current_time - self.last_switch_time.get(node, 0)
            phase = self.junction_phases[node]

            if phase == 'GO':
                if elapsed > TRAFFIC_LIGHT_GO_TIME:
                    self.junction_phases[node] = 'YELLOW'
                    self.last_switch_time[node] = current_time
                    current_idx = self.current_green_index[node]
                    self.light_states[node][neighbors[current_idx]] = 'YELLOW'

            elif phase == 'YELLOW':
                if elapsed > TRAFFIC_LIGHT_YELLOW_TIME:
                    self.junction_phases[node] = 'COOLDOWN'
                    self.last_switch_time[node] = current_time
                    current_idx = self.current_green_index[node]
                    self.light_states[node][neighbors[current_idx]] = 'RED'

            elif phase == 'COOLDOWN':
                if elapsed > TRAFFIC_LIGHT_COOLDOWN_TIME:
                    self.junction_phases[node] = 'GO'
                    self.last_switch_time[node] = current_time
                    next_idx = (self.current_green_index[node] + 1) % len(neighbors)
                    self.current_green_index[node] = next_idx
                    self.light_states[node][neighbors[next_idx]] = 'GREEN'

    def get_time_remaining(self, node):
        elapsed = time.time() - self.last_switch_time.get(node, 0)
        phase = self.junction_phases.get(node, 'GO')

        if phase == 'GO':
            return max(0, TRAFFIC_LIGHT_GO_TIME - elapsed)
        elif phase == 'YELLOW':
            return max(0, TRAFFIC_LIGHT_YELLOW_TIME - elapsed)
        else:
            return max(0, TRAFFIC_LIGHT_COOLDOWN_TIME - elapsed)


def update_car_states(cars, traffic_light_manager, path_manager):
    traffic_light_manager.update()
    scale = traffic_light_manager.sim_map.render_scale

    # Scaled thresholds
    scaled_min_gap = MIN_GAP * scale
    scaled_stop_line = STOP_LINE_DIST * scale

    # Collect obstacles
    stop_obstacles = []
    for j_node in traffic_light_manager.junctions:
        for in_node, state in traffic_light_manager.light_states[j_node].items():
            path_key = (in_node, j_node)
            path = path_manager.get_path_for_nodes(path_key)
            if path:
                stop_obstacles.append(StopObstacle(path[-1].x, path[-1].y, path_key, state))

    # --- 1. GLOBAL JUNCTION LOCK (Prevent Gridlock) ---
    occupied_junctions = set()
    for c in cars:
        if c.current_node_sequence and len(c.current_node_sequence) == 3:
            occupied_junctions.add(c.current_node_sequence[1])

    # --- 2. RESET ALL CARS TO "GO" ---
    for car in cars:
        car.is_waiting = False
        car.target_speed = car.max_speed

        # --- 3. CHECK CONDITIONS ---
    for i, car in enumerate(cars):
        car_path_key = car.current_node_sequence
        if not car_path_key: continue

        is_turning = len(car_path_key) == 3
        my_front_extent = car.length * scale

        heading = pygame.math.Vector2(cos(car.angle), sin(car.angle))
        right_normal = pygame.math.Vector2(-sin(car.angle), cos(car.angle))

        # --- A. TRAFFIC LIGHTS (Only if NOT turning) ---
        if not is_turning:
            for stop in stop_obstacles:
                if stop.path_key == car_path_key:
                    if car.detailed_path:
                        stop_line_pos = car.detailed_path[-1]
                        center_dist = pygame.math.Vector2(car.x, car.y).distance_to(stop_line_pos)
                        dist_to_stop = center_dist - my_front_extent

                        if dist_to_stop > -5:
                            should_stop = False

                            if stop.state == 'RED':
                                should_stop = True
                            elif stop.state == 'YELLOW':
                                if dist_to_stop > scaled_stop_line:
                                    should_stop = True
                            elif stop.state == 'GREEN':
                                # Global Lock: If junction has ANY car, wait.
                                if dist_to_stop > 0 and stop.path_key[1] in occupied_junctions:
                                    should_stop = True

                            if should_stop and dist_to_stop < scaled_stop_line:
                                car.target_speed = 0  # INSTANT STOP
                                car.is_waiting = True
                                break

        if car.is_waiting: continue

        # --- B. CAR FOLLOWING (LASER & SEPARATION) ---
        my_pos = pygame.math.Vector2(car.x, car.y)

        for j, other_car in enumerate(cars):
            if i == j: continue

            other_pos = pygame.math.Vector2(other_car.x, other_car.y)
            to_other = other_pos - my_pos
            dist_sq = to_other.length_squared()

            # --- 1. GOD MODE SEPARATION (Fixes Overlaps) ---
            touch_dist = (car.length * scale + other_car.length * scale) * 0.6
            if dist_sq < touch_dist ** 2:
                if dist_sq == 0:
                    if i > j:
                        car.target_speed = 0
                        car.is_waiting = True
                        break
                    continue

                # Check who is behind
                if heading.dot(to_other) > 0:  # I am behind
                    car.target_speed = 0  # FREEZE
                    car.is_waiting = True

                    # Teleport back to resolve overlap
                    push = to_other.normalize() * -2
                    car.x += push.x
                    car.y += push.y
                    break
                else:
                    continue

            # --- 2. THE LASER (Visual Stop) ---
            if is_turning: continue

            forward_dist = to_other.dot(heading)
            side_dist = to_other.dot(right_normal)

            other_radius = (other_car.length * scale) / 2
            gap = forward_dist - (my_front_extent + other_radius)

            lane_width = 14 * scale

            if abs(side_dist) < (lane_width / 2) and forward_dist > 0:
                if gap < scaled_min_gap:
                    car.target_speed = 0  # INSTANT STOP
                    car.is_waiting = True
                    break