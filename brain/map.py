# this is simply for the map. we start with a simple grid map of 4 squares to make it simple. 4 tjunctions + 1 4way should be simple enough.
import pygame

# --- Constants ---
ROAD_WIDTH = 80  # The total width of a two-lane road
LANE_WIDTH = ROAD_WIDTH // 2
DASH_LENGTH = 20

# --- Colors ---
GRAY = (100, 100, 100)
YELLOW = (255, 255, 0)


class Map:
    """
    Defines the road network for the simulation.
    It holds the data for nodes (intersections) and roads,
    and is responsible for drawing the map and providing pathing information.
    """

    def __init__(self):
        # Nodes are defined as a dictionary with a name and a (x, y) position.
        # This 3x3 grid gives us corners, T-junctions, and a 4-way intersection.
        self.nodes = {
            'A': (150, 125), 'B': (400, 125), 'C': (650, 125),
            'D': (150, 300), 'E': (400, 300), 'F': (650, 300),
            'G': (150, 475), 'H': (400, 475), 'I': (650, 475),
        }

        # Roads connect two nodes. The list contains tuples of node names.
        self.roads = [
            ('A', 'B'), ('B', 'C'),
            ('A', 'D'), ('B', 'E'), ('C', 'F'),
            ('D', 'E'), ('E', 'F'),
            ('D', 'G'), ('E', 'H'), ('F', 'I'),
            ('G', 'H'), ('H', 'I'),
        ]

    def get_lane_centerline(self, start_node_name, end_node_name):
        """
        Calculates the start and end points of the right-hand lane on a road.
        This is key to making cars follow a specific lane.

        :param start_node_name: The name of the starting node for the car's travel.
        :param end_node_name: The name of the ending node for the car's travel.
        :return: A tuple of (start_vector, end_vector) for the lane.
        """
        # Ensure the road exists in either direction
        is_forward = (start_node_name, end_node_name) in self.roads
        is_backward = (end_node_name, start_node_name) in self.roads
        if not is_forward and not is_backward:
            return None, None

        start_pos = pygame.math.Vector2(self.nodes[start_node_name])
        end_pos = pygame.math.Vector2(self.nodes[end_node_name])

        direction = (end_pos - start_pos).normalize()
        # Get a vector perpendicular to the road's direction for the offset
        perpendicular = direction.rotate(90)

        # Offset from the center line to the middle of the right-hand lane
        offset = perpendicular * (LANE_WIDTH / 2)

        # The lane runs from the start to the end, offset to the right
        return start_pos + offset, end_pos + offset

    def draw(self, screen):
        """
        Draws all the roads and intersections on the screen.
        """
        # Draw roads first
        for start_node, end_node in self.roads:
            start_pos = self.nodes[start_node]
            end_pos = self.nodes[end_node]
            self._draw_road_segment(screen, start_pos, end_pos)

        # Draw intersection squares on top of roads for a cleaner look
        for node_pos in self.nodes.values():
            # Center the intersection block on the node
            left = node_pos[0] - LANE_WIDTH
            top = node_pos[1] - LANE_WIDTH
            pygame.draw.rect(screen, GRAY, (left, top, ROAD_WIDTH, ROAD_WIDTH))

    def _draw_road_segment(self, screen, start_pos, end_pos):
        """
        Draws a single two-lane road segment with a dashed center line.
        """
        # Draw the main road surface as a thick line
        pygame.draw.line(screen, GRAY, start_pos, end_pos, ROAD_WIDTH)

        # Draw the dashed yellow line for two lanes
        line_vector = pygame.math.Vector2(end_pos) - pygame.math.Vector2(start_pos)
        line_length = line_vector.length()

        if line_length == 0: return  # Avoid division by zero

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