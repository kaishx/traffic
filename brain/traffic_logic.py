import pygame
import time
from collections import namedtuple
from math import sqrt, cos, sin, pi

# A simple object representing a stop point for a red light.
StopObstacle = namedtuple('StopObstacle', ['x', 'y', 'path_key'])

# --- TIMING CONFIGURATION ---
TRAFFIC_LIGHT_GO_TIME = 5.0  # Green light duration
TRAFFIC_LIGHT_COOLDOWN_TIME = 1  # All-Red duration between switches

CAR_FOLLOW_DISTANCE = 35
CAR_BOUNDING_BOX_SIZE = 20
CAR_LENGTH = 10


class TrafficLightManager:
    def __init__(self, sim_map):
        self.sim_map = sim_map
        self.junctions = [node for node, nbrs in sim_map.graph.adjacency_list.items() if len(nbrs) >= 3]

        self.light_states = {}
        self.last_switch_time = {}
        self.current_green_index = {}

        # Track which phase the junction is in: 'GO' or 'COOLDOWN'
        self.junction_phases = {}

        for node in self.junctions:
            self.last_switch_time[node] = time.time()
            self.junction_phases[node] = 'GO'  # Start in GO mode

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

            # --- PHASE 1: GREEN LIGHT (GO) ---
            if phase == 'GO':
                if elapsed > TRAFFIC_LIGHT_GO_TIME:
                    # Switch to COOLDOWN (All Red)
                    self.junction_phases[node] = 'COOLDOWN'
                    self.last_switch_time[node] = current_time

                    # Turn the current Green to Red
                    current_idx = self.current_green_index[node]
                    current_nbr = neighbors[current_idx]
                    self.light_states[node][current_nbr] = 'RED'

            # --- PHASE 2: CLEARANCE (COOLDOWN) ---
            elif phase == 'COOLDOWN':
                if elapsed > TRAFFIC_LIGHT_COOLDOWN_TIME:
                    # Switch to next GO (Next Green)
                    self.junction_phases[node] = 'GO'
                    self.last_switch_time[node] = current_time

                    # Advance to next neighbor
                    next_idx = (self.current_green_index[node] + 1) % len(neighbors)
                    self.current_green_index[node] = next_idx

                    # Turn the new neighbor to Green
                    next_nbr = neighbors[next_idx]
                    self.light_states[node][next_nbr] = 'GREEN'

    def get_time_remaining(self, node):
        """Returns time remaining for the CURRENT phase (either Go or Cooldown)."""
        elapsed = time.time() - self.last_switch_time.get(node, 0)
        phase = self.junction_phases.get(node, 'GO')

        if phase == 'GO':
            return max(0, TRAFFIC_LIGHT_GO_TIME - elapsed)
        else:
            # During cooldown, maybe show 0 or the cooldown time?
            # Let's show the cooldown time so you know when Green is coming.
            return max(0, TRAFFIC_LIGHT_COOLDOWN_TIME - elapsed)


def update_car_states(cars, traffic_light_manager, path_manager):
    """
    Updates car states with improved 'Distance-Based' queuing to prevent overlapping.
    """
    traffic_light_manager.update()

    # 1. Update Red Light Obstacles
    stop_obstacles = []
    for j_node in traffic_light_manager.junctions:
        for in_node, state in traffic_light_manager.light_states[j_node].items():
            if state == 'RED':
                path_key = (in_node, j_node)
                path = path_manager.get_path_for_nodes(path_key)
                if path:
                    stop_obstacles.append(StopObstacle(path[-1].x, path[-1].y, path_key))

    # 2. Reset Wait States
    for car in cars:
        car.resume()

    # 3. Check Conditions
    for i, car in enumerate(cars):
        car_path_key = car.current_node_sequence
        if not car_path_key: continue

        # --- A. Red Light Check ---
        for stop in stop_obstacles:
            if stop.path_key == car_path_key:
                if car.detailed_path:
                    stop_line_pos = car.detailed_path[-1]
                    dist_to_stop = pygame.math.Vector2(car.x, car.y).distance_to(stop_line_pos)

                    if dist_to_stop < CAR_FOLLOW_DISTANCE:
                        car.wait()
                        break

        if car.is_waiting: continue

        # --- B. Car-Following Logic (Prevents Overlap) ---
        my_pos = pygame.math.Vector2(car.x, car.y)

        for j, other_car in enumerate(cars):
            if i == j: continue

            other_pos = pygame.math.Vector2(other_car.x, other_car.y)
            distance = my_pos.distance_to(other_pos)

            # 1. CRITICAL FIX: Ghost Bug
            if distance < 5:
                car.wait()
                break

            # 2. Standard Following Check
            if distance < CAR_FOLLOW_DISTANCE:

                # Determine if the other car is "Ahead"
                to_other = other_pos - my_pos
                if to_other.length() == 0: continue

                # Normalize manually
                to_other = to_other.normalize()

                # Dot product check
                forward_dot = cos(car.angle) * to_other.x + sin(car.angle) * to_other.y

                if forward_dot > 0.7:
                    car.wait()
                    break