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
RED = (255, 0, 0)
GREEN = (0, 255, 0)
RAIL_COLOR = (255, 0, 255)  # Magenta for rails


class Graph:
    def __init__(self, nodes, roads):
        self.nodes = nodes
        self.roads = roads
        self.adjacency_list = {node: [] for node in self.nodes}
        for road in self.roads:
            self.adjacency_list[road[0]].append(road[1])
            self.adjacency_list[road[1]].append(road[0])

    def get_neighbors(self, node, previous_node=None):
        neighbors = self.adjacency_list[node]
        if previous_node is None:
            return neighbors
        return [n for n in neighbors if n != previous_node]

    def get_cost(self, from_node, to_node):
        return dist(self.nodes[from_node], self.nodes[to_node])


class Map:
    def __init__(self):
        self.LANE_WIDTH = LANE_WIDTH
        self.nodes = {
            1: (150, 120), 2: (400, 120), 3: (650, 120),
            4: (150, 300), 5: (400, 300), 6: (650, 300),
            7: (150, 480), 8: (400, 480), 9: (650, 480),
        }
        self.roads = [
            (1, 2), (2, 3), (1, 4), (2, 5), (3, 6),
            (4, 5), (5, 6), (4, 7), (5, 8), (6, 9),
            (7, 8), (8, 9),
        ]
        self.graph = Graph(self.nodes, self.roads)
        self.font = pygame.font.SysFont("Arial", 20, bold=True)
        self.small_font = pygame.font.SysFont("Arial", 16)

    def draw(self, screen, traffic_light_manager, path_manager, view_mode):
        """Draws map elements based on view_mode."""

        # 1. Base Roads (Always visible)
        for start_node, end_node in self.roads:
            self._draw_road_segment(screen, self.nodes[start_node], self.nodes[end_node])
        for node_pos in self.nodes.values():
            pygame.draw.rect(screen, GRAY,
                             (node_pos[0] - self.LANE_WIDTH, node_pos[1] - self.LANE_WIDTH, ROAD_WIDTH, ROAD_WIDTH))

        # 2. Rails (DEBUG Only)
        # Import ViewMode locally to check, or just assume integer 1
        if view_mode == 1:  # DEBUG
            if path_manager:
                for path in path_manager.paths.values():
                    if len(path) > 1:
                        pygame.draw.lines(screen, RAIL_COLOR, False, path, 1)

        # 3. Traffic Lights & Timers
        if traffic_light_manager:
            for junction_node in traffic_light_manager.junctions:

                # Draw Time Remaining (DEBUG & INFO)
                if view_mode in [1, 2]:
                    time_left = traffic_light_manager.get_time_remaining(junction_node)
                    time_text = f"{time_left:.1f}s"
                    text_surf = self.small_font.render(time_text, True, BLACK)
                    # Draw near the junction center
                    j_pos = self.nodes[junction_node]
                    screen.blit(text_surf, (j_pos[0] - 15, j_pos[1] - 10))

                # Draw Light Gates (All Modes)
                for inflow_node, state in traffic_light_manager.light_states[junction_node].items():
                    junction_pos = pygame.math.Vector2(self.nodes[junction_node])
                    inflow_pos = pygame.math.Vector2(self.nodes[inflow_node])

                    dir_vec = junction_pos - inflow_pos
                    if dir_vec.length() == 0: continue
                    unit_vec = dir_vec.normalize()
                    perp_vec = unit_vec.rotate(90)

                    stop_line_base = junction_pos - unit_vec * self.LANE_WIDTH
                    start_pos = stop_line_base - perp_vec * self.LANE_WIDTH
                    end_pos = stop_line_base

                    color = RED if state == 'RED' else GREEN
                    pygame.draw.line(screen, color, start_pos, end_pos, 5)

        # 4. Node Numbers (DEBUG Only)
        if view_mode == 1:
            for node_id, pos in self.nodes.items():
                img = self.font.render(str(node_id), True, BLACK)
                rect = img.get_rect(center=pos)
                screen.blit(img, rect)

    def _draw_road_segment(self, screen, start_pos, end_pos):
        # Unchanged
        pygame.draw.line(screen, GRAY, start_pos, end_pos, ROAD_WIDTH)
        line_vector = pygame.math.Vector2(end_pos) - pygame.math.Vector2(start_pos)
        line_length = line_vector.length()
        if line_length == 0: return
        unit_vector = line_vector.normalize()
        DASH_MARGIN = self.LANE_WIDTH
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
        ARROW_SPACING = 60
        ARROW_LENGTH = 12
        ARROW_WIDTH = 8
        ARROW_TAIL_LENGTH = 10
        ARROW_TAIL_WIDTH = 2
        ARROW_COLOR = BLACK
        END_MARGIN = 80
        perp = unit_vector.rotate(-90)
        lane_offset = self.LANE_WIDTH / 2
        left_lane_offset = perp * lane_offset
        right_lane_offset = -perp * lane_offset

        def _draw_arrow(center, direction, color=ARROW_COLOR):
            dir_unit = pygame.math.Vector2(direction).normalize()
            tip = center + dir_unit * (ARROW_LENGTH / 2)
            base_center = center - dir_unit * (ARROW_LENGTH / 2)
            perp_dir = dir_unit.rotate(-90)
            left_base = base_center + perp_dir * (ARROW_WIDTH / 2)
            right_base = base_center - perp_dir * (ARROW_WIDTH / 2)
            pygame.draw.polygon(screen, color, [tip, left_base, right_base])
            tail_back_center = base_center - dir_unit * ARROW_TAIL_LENGTH
            tail_half = ARROW_TAIL_WIDTH / 2
            tail_front_left = base_center + perp_dir * tail_half
            tail_front_right = base_center - perp_dir * tail_half
            tail_back_left = tail_back_center + perp_dir * tail_half
            tail_back_right = tail_back_center - perp_dir * tail_half
            pygame.draw.polygon(screen, color, [tail_front_left, tail_back_left, tail_back_right, tail_front_right])

        dist = END_MARGIN
        while dist < line_length - END_MARGIN:
            base_point = pygame.math.Vector2(start_pos) + unit_vector * dist
            pos_left = base_point + left_lane_offset
            _draw_arrow(pos_left, unit_vector)
            pos_right = base_point + right_lane_offset
            _draw_arrow(pos_right, -unit_vector)
            dist += ARROW_SPACING