import pygame
from math import dist

# --- Constants ---
ROAD_WIDTH = 80
LANE_WIDTH = ROAD_WIDTH // 2
DASH_LENGTH = 20

# --- Colors ---
GRAY = (100, 100, 100)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


class Graph:
    """Represents the road network as a graph for pathfinding."""
    def __init__(self, nodes, roads):
        self.nodes = nodes
        self.roads = roads
        self.adjacency_list = {node: [] for node in self.nodes}
        for road in self.roads:
            self.adjacency_list[road[0]].append(road[1])
            self.adjacency_list[road[1]].append(road[0])

    def get_neighbors(self, node, previous_node=None):
        """
        Gets the neighbors of a node, preventing 180-degree turns.
        A 180-degree turn is prevented by not allowing the path to return to the node
        it just came from during the A* search.
        """
        neighbors = self.adjacency_list[node]
        if previous_node is None:
            return neighbors
        
        # Filter out the previous node to prevent immediate U-turns in pathfinding.
        return [n for n in neighbors if n != previous_node]

    def get_cost(self, from_node, to_node):
        return dist(self.nodes[from_node], self.nodes[to_node])


class Map:
    """
    Defines the road network for the simulation.
    Holds the data for nodes and roads, and is responsible for drawing the map.
    """
    def __init__(self):
        self.LANE_WIDTH = LANE_WIDTH  # Expose for car logic
        # Nodes scaled inward (~70% of the original distance from map center)
        # to make roads shorter while maintaining road width
        self.nodes = {
            1: (190, 153),  2: (400, 153),  3: (610, 153),
            4: (190, 300),  5: (400, 300),  6: (610, 300),
            7: (190, 447),  8: (400, 447),  9: (610, 447),
        }
        self.roads = [
            (1, 2), (2, 3), (1, 4), (2, 5), (3, 6),
            (4, 5), (5, 6), (4, 7), (5, 8), (6, 9),
            (7, 8), (8, 9),
        ]
        self.graph = Graph(self.nodes, self.roads)
        
        # Initialize gate states for each node-road junction (default 'UP')
        # Each gate is identified by (node, neighbor) representing the gate positioned at `node`
        # for the road connecting `node` to `neighbor`.
        self.gates = {}
        for a, b in self.roads:
            self.gates[(a, b)] = 'UP'
            self.gates[(b, a)] = 'UP'
        
        # Font for drawing node numbers
        self.font = pygame.font.SysFont(None, 30)

    def draw(self, screen):
        """Draws all the roads, intersections, and node numbers on the screen."""
        # Draw roads first
        for start_node, end_node in self.roads:
            start_pos = self.nodes[start_node]
            end_pos = self.nodes[end_node]
            self._draw_road_segment(screen, start_pos, end_pos)

        # Draw intersection squares
        for node_pos in self.nodes.values():
            left = node_pos[0] - self.LANE_WIDTH
            top = node_pos[1] - self.LANE_WIDTH
            pygame.draw.rect(screen, GRAY, (left, top, ROAD_WIDTH, ROAD_WIDTH))
            
        # Draw gates at each road junction (2px thick lines; red when UP, translucent green when DOWN)
        gate_surf = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        for node_id, node_pos in self.nodes.items():
            neighbors = self.graph.get_neighbors(node_id)
            for nbr in neighbors:
                nbr_pos = self.nodes[nbr]
                dir_vec = pygame.math.Vector2(nbr_pos) - pygame.math.Vector2(node_pos)
                if dir_vec.length() == 0:
                    continue
                unit = dir_vec.normalize()
                # Position gate slightly outside the intersection square along the road
                gate_pos = pygame.math.Vector2(node_pos) + unit * (self.LANE_WIDTH + 4)
                perp = unit.rotate(-90)
                # Make gate span the road width
                line_len = ROAD_WIDTH - 4
                start = gate_pos + perp * (line_len / 2)
                end = gate_pos - perp * (line_len / 2)
                state = self.gates.get((node_id, nbr), 'UP')
                if state == 'UP':
                    color = (255, 0, 0, 255)
                else:
                    color = (0, 255, 0, 128)
                pygame.draw.line(gate_surf, color, start, end, 2)
        screen.blit(gate_surf, (0, 0))

        # Draw node numbers on top
        for node_id, pos in self.nodes.items():
            img = self.font.render(str(node_id), True, BLACK)
            # Center the text on the node
            rect = img.get_rect(center=pos)
            screen.blit(img, rect) 

    def _draw_road_segment(self, screen, start_pos, end_pos):
        """Draws a single two-lane road segment with a dashed center line and lane direction arrows (left-hand driving)."""
        pygame.draw.line(screen, GRAY, start_pos, end_pos, ROAD_WIDTH)

        line_vector = pygame.math.Vector2(end_pos) - pygame.math.Vector2(start_pos)
        line_length = line_vector.length()
        if line_length == 0: return
        unit_vector = line_vector.normalize()

        # Draw dashed center line, leaving margins near intersections
        DASH_MARGIN = self.LANE_WIDTH  # gap from each junction where dashes won't be drawn
        dash_start = DASH_MARGIN
        dash_end = line_length - DASH_MARGIN
        if dash_end > dash_start:
            distance = dash_start
            draw_dash = True
            while distance < dash_end:
                dash_segment_end = min(distance + DASH_LENGTH, dash_end)
                if draw_dash:
                    start_dash = pygame.math.Vector2(start_pos) + unit_vector * distance
                    end_dash = pygame.math.Vector2(start_pos) + unit_vector * dash_segment_end
                    pygame.draw.line(screen, YELLOW, start_dash, end_dash, 2)
                distance += DASH_LENGTH
                draw_dash = not draw_dash

        # Draw directional arrows along each lane to indicate left-side driving
        ARROW_SPACING = 60   # space between arrows along the road (reduced to make arrows denser)
        ARROW_LENGTH = 12
        ARROW_WIDTH = 8
        ARROW_TAIL_LENGTH = 10
        ARROW_TAIL_WIDTH = 2  # thinner tail
        ARROW_COLOR = BLACK
        END_MARGIN = 80  # don't draw arrows too close to intersections (increased to keep arrows away from nodes)

        # Perpendicular vector (left relative to the direction from start->end)
        perp = unit_vector.rotate(-90)

        # Offset lanes toward the edges: place arrows at 1/4 of the road width from center
        # 1/4 road width = half of a lane width (self.LANE_WIDTH / 2)
        lane_offset = self.LANE_WIDTH / 2
        left_lane_offset = perp * lane_offset      # for traffic going start -> end (left-side driving)
        right_lane_offset = -perp * lane_offset    # for traffic going end -> start

        def _draw_arrow(center, direction, color=ARROW_COLOR):
            """Draw a triangular arrow with a tail centered at `center` pointing along `direction` vector."""
            dir_unit = pygame.math.Vector2(direction).normalize()
            # Tip of the arrow head
            tip = center + dir_unit * (ARROW_LENGTH / 2)
            # Base center of head (front of tail)
            base_center = center - dir_unit * (ARROW_LENGTH / 2)
            # Perpendicular for widths
            perp_dir = dir_unit.rotate(-90)
            left_base = base_center + perp_dir * (ARROW_WIDTH / 2)
            right_base = base_center - perp_dir * (ARROW_WIDTH / 2)
            # Arrow head
            pygame.draw.polygon(screen, color, [tip, left_base, right_base])
            # Tail: rectangle-like polygon extending back from base_center
            tail_back_center = base_center - dir_unit * ARROW_TAIL_LENGTH
            tail_half = ARROW_TAIL_WIDTH / 2
            tail_front_left = base_center + perp_dir * tail_half
            tail_front_right = base_center - perp_dir * tail_half
            tail_back_left = tail_back_center + perp_dir * tail_half
            tail_back_right = tail_back_center - perp_dir * tail_half
            pygame.draw.polygon(screen, color, [tail_front_left, tail_back_left, tail_back_right, tail_front_right])

        # Place arrows along the road, avoiding endpoints
        dist = END_MARGIN
        while dist < line_length - END_MARGIN:
            base_point = pygame.math.Vector2(start_pos) + unit_vector * dist
            # Arrow for traffic going start -> end (placed on the left side of the road)
            pos_left = base_point + left_lane_offset
            _draw_arrow(pos_left, unit_vector)
            # Arrow for traffic going end -> start (placed on the other lane)
            pos_right = base_point + right_lane_offset
            _draw_arrow(pos_right, -unit_vector)

            dist += ARROW_SPACING

    def set_gate(self, node, neighbor, state):
        """Set the gate state at `node` for the road to `neighbor`.
        State must be 'UP' (opaque red) or 'DOWN' (translucent green).
        """
        if state not in ('UP', 'DOWN'):
            raise ValueError("state must be 'UP' or 'DOWN'")
        if (node, neighbor) not in self.gates:
            raise KeyError(f"No gate corresponding to node {node} and neighbor {neighbor}")
        self.gates[(node, neighbor)] = state

    def toggle_gate(self, node, neighbor):
        """Toggle the gate between 'UP' and 'DOWN'."""
        if (node, neighbor) not in self.gates:
            raise KeyError(f"No gate corresponding to node {node} and neighbor {neighbor}")
        self.gates[(node, neighbor)] = 'DOWN' if self.gates[(node, neighbor)] == 'UP' else 'UP'

    def get_gate(self, node, neighbor):
        """Return the gate state ('UP' or 'DOWN')."""
        return self.gates.get((node, neighbor), 'UP')

    # NOTE: Traffic-level waiting/obstacle logic has been moved to
    # `traffic_logic.update_car_waiting_states(sim_map, cars)` to keep the Map
    # class focused on map data and drawing.