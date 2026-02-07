import pygame
from math import dist, atan2, cos, sin
from typing import List, Optional, Tuple
import heapq
from enum import Enum


# --- A* Pathfinding (Unchanged) ---
def a_star_search(start_node: str, end_node: str, graph: 'Graph', initial_previous_node: Optional[str] = None):
    frontier = [(0, start_node)]
    came_from = {start_node: initial_previous_node}
    cost_so_far = {start_node: 0}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == end_node: break
        previous = came_from.get(current)
        for neighbor in graph.get_neighbors(current, previous):
            new_cost = cost_so_far[current] + graph.get_cost(current, neighbor)
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(graph.nodes[neighbor], graph.nodes[end_node])
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current
    return reconstruct_path(came_from, start_node, end_node)


def heuristic(a, b): return dist(a, b)


def reconstruct_path(came_from: dict, start: str, end: str) -> List[str]:
    current = end
    path = []
    if end not in came_from: return []
    while current != start:
        path.append(current)
        current = came_from.get(current)
        if current is None: return []
    path.append(start)
    path.reverse()
    return path


class CarState(Enum):
    ACCELERATE = 1
    CRUISE = 2
    BRAKE = 3

# --- Car Logic (INSTANT PHYSICS UPDATE) ---
class Car:
    def __init__(self, map_data: 'Map', name: str, color: tuple,
                 max_speed: float = 69.0, length: float = 10.0,
                 speed_stop_factor: float = 0.16, engine_power: float = 375,
                 weight: float = 1500, brake_force: float = 110):
        self.map_data = map_data
        self.name = str(name)
        self.color = color

        # Physics Properties
        self.x, self.y = 0.0, 0.0
        self.length = float(length)
        self.max_speed = float(max_speed)
        self.speed_stop_factor = float(speed_stop_factor)
        self.engine_power = float(engine_power)
        self.weight = float(weight)
        self.brake_force = float(brake_force)

        # Dynamic State
        self.speed = self.max_speed
        self.state = CarState.ACCELERATE
        self.angle = 0.0

        # Pathing State
        self.node_path: List[str] = []
        self.node_path_index = 0
        self.detailed_path: List[pygame.math.Vector2] = []
        self.detailed_path_index = 0
        self.current_node_sequence: Optional[Tuple[str, ...]] = None
        self._pending_turn_sequence: Optional[Tuple[str, ...]] = None

    def set_path(self, node_path: List[str], path_manager):
        """Sets a new high-level path and generates the first detailed segment."""
        self.node_path = node_path
        self.node_path_index = 0
        self.detailed_path_index = 0
        self._pending_turn_sequence = None
        self.state = CarState.ACCELERATE

        self._generate_next_detailed_path_segment(path_manager)

        if self.detailed_path:
            self.x, self.y = self.detailed_path[0]
            if len(self.detailed_path) > 1:
                vec = self.detailed_path[1] - self.detailed_path[0]
                if vec.length() > 0:
                    self.angle = atan2(vec.y, vec.x)

    def _generate_next_detailed_path_segment(self, path_manager):
        if self._pending_turn_sequence:
            self.detailed_path = path_manager.get_path_for_nodes(self._pending_turn_sequence)
            self.current_node_sequence = self._pending_turn_sequence
            self._pending_turn_sequence = None
            self.node_path_index += 1
            self.detailed_path_index = 0
            return

        if self.node_path_index >= len(self.node_path) - 1:
            self.detailed_path = []
            self.state = CarState.BRAKE  # Stop at end of path
            self.current_node_sequence = None
            return

        current_node = self.node_path[self.node_path_index]
        next_node = self.node_path[self.node_path_index + 1]

        sequence = (current_node, next_node)
        self.detailed_path = path_manager.get_path_for_nodes(sequence)
        self.current_node_sequence = sequence
        self.detailed_path_index = 0

        if self.node_path_index < len(self.node_path) - 2:
            future_node = self.node_path[self.node_path_index + 2]
            self._pending_turn_sequence = (current_node, next_node, future_node)
        else:
            self._pending_turn_sequence = None

    def calc_brake_stop_distance(self):
        return (self.speed ** 2) / (2 * self.brake_force)

    def calc_cruise_distance(self, target_v, surface_friction=1.0):
        return (self.speed - target_v) / (self.speed_stop_factor * surface_friction)

    def update(self, dt, path_manager):
        if not self.detailed_path:
            return

        # --- REALISTIC PHYSICS ---
        surface_friction = 1.0

        speed_decayed = self.speed * (1 - self.speed_stop_factor * surface_friction * dt)

        engine = 0.0
        if self.state in [CarState.ACCELERATE]:
            if self.speed <= self.max_speed:
                engine = (self.engine_power / self.weight) * dt * self.max_speed * surface_friction

        braking = 0.0
        if self.state == CarState.BRAKE:
            braking = self.brake_force * surface_friction * dt

        self.speed = speed_decayed + engine - braking

        if self.speed < 0: self.speed = 0

        move_dist = self.speed * dt

        # --- MOVE ALONG DETAILED PATH ---

        while move_dist > 0 and self.detailed_path_index < len(self.detailed_path) - 1:
            target_pos = self.detailed_path[self.detailed_path_index + 1]
            segment_vec = target_pos - pygame.math.Vector2(self.x, self.y)
            segment_len = segment_vec.length()

            if segment_len == 0:
                self.detailed_path_index += 1
                continue

            if move_dist >= segment_len:
                self.x, self.y = target_pos
                move_dist -= segment_len
                self.detailed_path_index += 1
            else:
                self.angle = atan2(segment_vec.y, segment_vec.x)
                self.x += cos(self.angle) * move_dist
                self.y += sin(self.angle) * move_dist
                move_dist = 0

        if self.detailed_path_index >= len(self.detailed_path) - 1:
            self._generate_next_detailed_path_segment(path_manager)
