import pygame
import heapq
from math import cos, sin, dist, radians, atan2, pi
from typing import List, Optional
from enum import Enum, auto


# --- A* Pathfinding ---
def a_star_search(start_node: str, end_node: str, graph: 'Graph', initial_previous_node: Optional[str] = None):
    """
    Finds the shortest path between two nodes, avoiding 180-degree turns.
    The `initial_previous_node` is the node the car was at *before* the `start_node`
    of the current path segment. This prevents the path from immediately turning back.
    """
    frontier = [(0, start_node)]
    
    # The `came_from` dictionary now stores the path taken to reach a node.
    came_from = {start_node: initial_previous_node}
    cost_so_far = {start_node: 0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == end_node:
            break

        # Get the node we came from to reach the current node.
        previous = came_from.get(current)
        
        # Pass the previous node to `get_neighbors` to enforce turn restrictions.
        for neighbor in graph.get_neighbors(current, previous):
            new_cost = cost_so_far[current] + graph.get_cost(current, neighbor)
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(graph.nodes[neighbor], graph.nodes[end_node])
                heapq.heappush(frontier, (priority, neighbor))
                
                # Record the path taken to this neighbor.
                came_from[neighbor] = current
                
    return reconstruct_path(came_from, start_node, end_node)


def heuristic(a, b):
    return dist(a, b)


def reconstruct_path(came_from: dict, start: str, end: str) -> List[str]:
    """Reconstructs the path from the `came_from` dictionary, handling path segments correctly."""
    current = end
    path = []
    
    # If the end node is not in came_from, no path was found.
    if end not in came_from:
        return []
        
    # Trace the path backwards from end to start.
    while current != start:
        path.append(current)
        current = came_from.get(current)
        # If we hit a dead end before reaching the start, the path is invalid.
        if current is None:
            return []
            
    path.append(start)
    path.reverse()
    return path


# --- Final, Precise State-Machine Car Architecture ---

class CarState(Enum):
    DRIVING_STRAIGHT = auto()
    TURNING = auto()
    WAITING = auto()


class Car:
    """A car that follows a high-level node path using a simple, precise state machine."""

    def __init__(self, speed: float, map_data: 'Map', name: str = "Car"):
        self.x = 0.0
        self.y = 0.0
        self.speed = float(speed)
        self.angle = 0.0
        self.name = str(name)

        self.map_data = map_data
        self.node_path: List[str] = []
        self.path_index = 0

        self.state = CarState.DRIVING_STRAIGHT
        self.target_pos = None
        self._prev_state = None

        # Turn-specific attributes
        self.turn_center = None
        self.turn_radius = 0.0
        self.turn_direction = 0
        self.turn_angle_remaining = 0.0
        self.start_radial_angle = 0.0

    def set_path(self, node_path: List[str]):
        """Initializes the car's path and state."""
        self.node_path = node_path
        self.path_index = 0
        self._prepare_for_straight()

        if self.target_pos and len(self.node_path) > 1:
            p_start, _, _, _ = self._get_lane_vectors(self.node_path[0], self.node_path[1])
            self.x, self.y = p_start
            self.angle = atan2(self.target_pos.y - self.y, self.target_pos.x - self.x)

    def _get_lane_vectors(self, from_node, to_node):
        """Calculates lane points and vectors for left-hand traffic."""
        start_pos = pygame.math.Vector2(self.map_data.nodes[from_node])
        end_pos = pygame.math.Vector2(self.map_data.nodes[to_node])
        direction = (end_pos - start_pos).normalize()
        perpendicular = direction.rotate(-90)
        offset_vec = perpendicular * (self.map_data.LANE_WIDTH / 2)
        return start_pos + offset_vec, end_pos + offset_vec, direction, offset_vec

    def _prepare_for_straight(self):
        """Sets the target for the next straight segment."""
        if self.path_index >= len(self.node_path) - 1:
            self.speed = 0
            return

        start_node = self.node_path[self.path_index]
        end_node = self.node_path[self.path_index + 1]
        _, self.target_pos, _, _ = self._get_lane_vectors(start_node, end_node)
        self.state = CarState.DRIVING_STRAIGHT

    def _prepare_for_turn(self):
        """
        **THE FINAL, CORRECTED PIVOT LOGIC**
        Calculates the geometry for the upcoming turn with perfect precision.
        """
        if self.path_index >= len(self.node_path) - 2:
            self._prepare_for_straight()
            return

        prev_node = self.node_path[self.path_index]
        center_node = self.node_path[self.path_index + 1]
        next_node = self.node_path[self.path_index + 2]

        _, p_end, v_dir, _ = self._get_lane_vectors(prev_node, center_node)
        _, _, v_next_dir, _ = self._get_lane_vectors(center_node, next_node)

        if abs(v_dir.dot(v_next_dir)) > 0.99:  # Straight through
            self.path_index += 1
            self._prepare_for_straight()
            return

        cross_product = v_dir.x * v_next_dir.y - v_dir.y * v_next_dir.x

        # The car's current position when it decides to turn
        current_car_pos = pygame.math.Vector2(self.x, self.y)
        current_car_heading_vec = pygame.math.Vector2(cos(self.angle), sin(self.angle))

        if cross_product > 0:  # Left Turn (Counter-Clockwise)
            self.turn_direction = 1
            self.turn_radius = self.map_data.LANE_WIDTH * 1.5  # Wide turn for left
            # Pivot point is to the LEFT of the car's current heading
            self.turn_center = current_car_pos + current_car_heading_vec.rotate(90) * self.turn_radius
        else:  # Right Turn (Clockwise)
            self.turn_direction = -1
            self.turn_radius = self.map_data.LANE_WIDTH / 2  # Tight turn for right
            # Pivot point is to the RIGHT of the car's current heading
            self.turn_center = current_car_pos + current_car_heading_vec.rotate(-90) * self.turn_radius

        self.turn_angle_remaining = pi / 2

        start_vec = current_car_pos - self.turn_center
        self.start_radial_angle = atan2(start_vec.y, start_vec.x)

        self.state = CarState.TURNING

    def wait(self):
        """Put the car into WAITING state (it will stop moving until resumed)."""
        if self.state != CarState.WAITING:
            self._prev_state = self.state
            self.state = CarState.WAITING

    def resume(self):
        """Resume the previous state after waiting."""
        if self.state == CarState.WAITING:
            self.state = self._prev_state or CarState.DRIVING_STRAIGHT
            self._prev_state = None

    def update(self, dt: float):
        """The final state machine with precision fixes."""
        if self.speed == 0 or self.state == CarState.WAITING:
            return

        move_dist = self.speed * dt

        if self.state == CarState.DRIVING_STRAIGHT:
            # Dynamic look-ahead based on speed, with a minimum value.
            look_ahead = max(20, self.speed * 0.4)
            if self.path_index < len(self.node_path) - 2 and dist((self.x, self.y), self.target_pos) < look_ahead:
                self._prepare_for_turn()
            elif dist((self.x, self.y), self.target_pos) <= move_dist:
                self.x, self.y = self.target_pos
                self.path_index += 1
                self._prepare_for_straight()
            else:
                self.angle = atan2(self.target_pos.y - self.y, self.target_pos.x - self.x)
                self.x += cos(self.angle) * move_dist
                self.y += sin(self.angle) * move_dist

        elif self.state == CarState.TURNING:
            angular_velocity = self.speed / self.turn_radius
            angle_this_frame = angular_velocity * dt

            if angle_this_frame >= self.turn_angle_remaining:
                # FINISH THE TURN PERFECTLY
                angle_offset = self.turn_angle_remaining * self.turn_direction
                final_radial_angle = self.start_radial_angle + (
                            pi / 2 - self.turn_angle_remaining) * self.turn_direction + angle_offset

                self.x = self.turn_center.x + cos(final_radial_angle) * self.turn_radius
                self.y = self.turn_center.y + sin(final_radial_angle) * self.turn_radius

                self.path_index += 1
                self._prepare_for_straight()

                if self.target_pos:
                    self.angle = atan2(self.target_pos.y - self.y, self.target_pos.x - self.x)
            else:
                # CONTINUE THE TURN
                self.turn_angle_remaining -= angle_this_frame

                progress = (pi / 2 - self.turn_angle_remaining) / (pi / 2)
                angle_offset = (pi / 2 * progress) * self.turn_direction
                current_radial_angle = self.start_radial_angle + angle_offset

                self.x = self.turn_center.x + cos(current_radial_angle) * self.turn_radius
                self.y = self.turn_center.y + sin(current_radial_angle) * self.turn_radius

                self.angle = current_radial_angle + (pi / 2 * self.turn_direction)