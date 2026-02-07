import pygame
from math import dist

# --- Colors ---
GRAY = (100, 100, 100)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
RAIL_COLOR = (255, 0, 255)


class Graph:
    def __init__(self, nodes, roads):
        self.nodes = nodes
        self.roads = roads
        self.adjacency_list = {node: [] for node in self.nodes}
        for road in self.roads:
            self.adjacency_list[road[0]].append(road[1])
            self.adjacency_list[road[1]].append(road[0])

    def get_neighbors(self, node, previous_node=None):
        neighbors = self.adjacency_list.get(node, [])
        if previous_node is None:
            return neighbors
        return [n for n in neighbors if n != previous_node]

    def get_cost(self, from_node, to_node):
        return dist(self.nodes[from_node], self.nodes[to_node])


class Map:
    def __init__(self, screen_width, screen_height, grid_cols, grid_rows, active_nodes):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.cols = grid_cols
        self.rows = grid_rows
        self.active_nodes_set = set(active_nodes)

        self.cell_w = self.screen_width / self.cols
        self.cell_h = self.screen_height / self.rows

        min_dim = min(self.cell_w, self.cell_h)
        self.ROAD_WIDTH = int(min_dim * 0.35)
        self.ROAD_WIDTH = max(20, self.ROAD_WIDTH)
        self.LANE_WIDTH = self.ROAD_WIDTH // 2

        self.render_scale = min_dim / 150.0

        self.nodes = {}
        self.roads = []
        self._generate_grid()
        self.graph = Graph(self.nodes, self.roads)

        font_size = max(10, int(20 * self.render_scale))
        self.font = pygame.font.SysFont("Arial", font_size, bold=True)
        self.small_font = pygame.font.SysFont("Arial", max(8, int(16 * self.render_scale)))

    def _generate_grid(self):
        for node_id in self.active_nodes_set:
            idx = node_id - 1
            row = idx // self.cols
            col = idx % self.cols
            x = (col * self.cell_w) + (self.cell_w / 2)
            y = (row * self.cell_h) + (self.cell_h / 2)
            self.nodes[node_id] = (x, y)

        for node_id in self.active_nodes_set:
            idx = node_id - 1
            row = idx // self.cols
            col = idx % self.cols

            if col < self.cols - 1:
                right_neighbor = node_id + 1
                if right_neighbor in self.active_nodes_set:
                    self.roads.append((node_id, right_neighbor))

            if row < self.rows - 1:
                down_neighbor = node_id + self.cols
                if down_neighbor in self.active_nodes_set:
                    self.roads.append((node_id, down_neighbor))

    def draw(self, screen, traffic_light_manager, path_manager, view_mode):
        for start_node, end_node in self.roads:
            self._draw_road_segment(screen, self.nodes[start_node], self.nodes[end_node])

        for node_pos in self.nodes.values():
            pygame.draw.rect(screen, GRAY,
                             (node_pos[0] - self.LANE_WIDTH, node_pos[1] - self.LANE_WIDTH,
                              self.ROAD_WIDTH, self.ROAD_WIDTH))

        if view_mode == 1:
            if path_manager:
                for path in path_manager.paths.values():
                    if len(path) > 1:
                        pygame.draw.lines(screen, RAIL_COLOR, False, path, 1)

        if traffic_light_manager:
            for junction_node in traffic_light_manager.junctions:
                jm = traffic_light_manager.junction_managers[junction_node]
                if view_mode in [1, 2]:
                    time_left = jm.get_time_remaining()
                    time_text = f"{time_left:.1f}s"
                    text_surf = self.small_font.render(time_text, True, BLACK)
                    j_pos = self.nodes[junction_node]
                    screen.blit(text_surf, (j_pos[0] - 15, j_pos[1] - 10))

                for inflow_node, state in jm.light_states.items():
                    junction_pos = pygame.math.Vector2(self.nodes[junction_node])
                    inflow_pos = pygame.math.Vector2(self.nodes[inflow_node])

                    dir_vec = junction_pos - inflow_pos
                    if dir_vec.length() == 0: continue
                    unit_vec = dir_vec.normalize()
                    perp_vec = unit_vec.rotate(90)

                    stop_line_base = junction_pos - unit_vec * self.LANE_WIDTH
                    start_pos = stop_line_base - perp_vec * self.LANE_WIDTH
                    end_pos = stop_line_base

                    # Determine Color
                    if state == 'RED':
                        color = RED
                    elif state == 'YELLOW':
                        color = YELLOW
                    else:
                        color = GREEN

                    line_width = max(2, int(5 * self.render_scale))
                    pygame.draw.line(screen, color, start_pos, end_pos, line_width)

        if view_mode == 1:
            for node_id, pos in self.nodes.items():
                img = self.font.render(str(node_id), True, BLACK)
                rect = img.get_rect(center=pos)
                screen.blit(img, rect)

    def _draw_road_segment(self, screen, start_pos, end_pos):
        pygame.draw.line(screen, GRAY, start_pos, end_pos, self.ROAD_WIDTH)

        line_vector = pygame.math.Vector2(end_pos) - pygame.math.Vector2(start_pos)
        line_length = line_vector.length()
        if line_length == 0: return
        unit_vector = line_vector.normalize()

        dash_len = 20 * self.render_scale

        dist = self.LANE_WIDTH
        while dist < line_length - self.LANE_WIDTH:
            start_dash = pygame.math.Vector2(start_pos) + unit_vector * dist
            end_dash = start_dash + unit_vector * dash_len
            if (pygame.math.Vector2(start_pos).distance_to(end_dash) <
                    pygame.math.Vector2(start_pos).distance_to(
                        pygame.math.Vector2(end_pos) - unit_vector * self.LANE_WIDTH)):
                pygame.draw.line(screen, YELLOW, start_dash, end_dash, max(1, int(2 * self.render_scale)))
            dist += dash_len * 2

        arrow_spacing = 60 * self.render_scale
        arrow_len = 12 * self.render_scale
        arrow_width = 8 * self.render_scale

        dist = 80 * self.render_scale
        perp = unit_vector.rotate(-90)
        lane_offset = self.LANE_WIDTH / 2

        while dist < line_length - (80 * self.render_scale):
            base_point = pygame.math.Vector2(start_pos) + unit_vector * dist
            self._draw_arrow(screen, base_point + perp * lane_offset, unit_vector, arrow_len, arrow_width)
            self._draw_arrow(screen, base_point - perp * lane_offset, -unit_vector, arrow_len, arrow_width)
            dist += arrow_spacing

    def _draw_arrow(self, screen, center, direction, length, width):
        dir_unit = direction
        tip = center + dir_unit * (length / 2)
        base = center - dir_unit * (length / 2)
        perp = dir_unit.rotate(-90)
        left = base + perp * (width / 2)
        right = base - perp * (width / 2)
        pygame.draw.polygon(screen, BLACK, [tip, left, right])