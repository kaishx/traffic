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
        self.nodes = {
            1: (150, 125), 2: (400, 125), 3: (650, 125),
            4: (150, 300), 5: (400, 300), 6: (650, 300),
            7: (150, 475), 8: (400, 475), 9: (650, 475),
        }
        self.roads = [
            (1, 2), (2, 3), (1, 4), (2, 5), (3, 6),
            (4, 5), (5, 6), (4, 7), (5, 8), (6, 9),
            (7, 8), (8, 9),
        ]
        self.graph = Graph(self.nodes, self.roads)
        
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
            
        # Draw node numbers on top
        for node_id, pos in self.nodes.items():
            img = self.font.render(str(node_id), True, BLACK)
            # Center the text on the node
            rect = img.get_rect(center=pos)
            screen.blit(img, rect)

    def _draw_road_segment(self, screen, start_pos, end_pos):
        """Draws a single two-lane road segment with a dashed center line."""
        pygame.draw.line(screen, GRAY, start_pos, end_pos, ROAD_WIDTH)

        line_vector = pygame.math.Vector2(end_pos) - pygame.math.Vector2(start_pos)
        line_length = line_vector.length()
        if line_length == 0: return
        unit_vector = line_vector.normalize()

        current_pos = pygame.math.Vector2(start_pos)
        distance_covered = 0
        is_drawing_dash = True
        while distance_covered < line_length:
            if is_drawing_dash:
                dash_end_dist = min(distance_covered + DASH_LENGTH, line_length)
                start_dash = current_pos
                end_dash = pygame.math.Vector2(start_pos) + unit_vector * dash_end_dist
                pygame.draw.line(screen, YELLOW, start_dash, end_dash, 2)
            distance_covered += DASH_LENGTH
            current_pos = pygame.math.Vector2(start_pos) + unit_vector * distance_covered
            is_drawing_dash = not is_drawing_dash
