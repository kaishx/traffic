import pygame
from math import atan2, sin, cos, pi


class PathManager:
    def __init__(self, sim_map):
        self.map = sim_map
        self.paths = self._generate_all_paths()

    def _generate_all_paths(self):
        """
        Generates paths for Left-Hand Traffic (LHT) using Corner Pivots.

        GEOMETRY RULES:
        1. LHT: Cars drive on the left side of the road vector.
        2. Left Turn (Tight): Pivots around the LEFT inner corner.
        3. Right Turn (Wide): Pivots around the RIGHT inner corner.
        """
        paths = {}
        nodes = self.map.nodes
        lane_width = self.map.LANE_WIDTH
        road_width = lane_width * 2
        junction_half_size = road_width / 2

        for start_id in nodes:
            neighbors = self.map.graph.get_neighbors(start_id)
            for junc_id in neighbors:

                # --- 1. Calculate Entry Geometry (LHT) ---
                p_start = pygame.math.Vector2(nodes[start_id])
                p_junc = pygame.math.Vector2(nodes[junc_id])

                # Vector pointing into the junction
                vec_in = (p_junc - p_start).normalize()

                # Perpendicular vector pointing to the LEFT of the car (for LHT)
                # (x, y) -> (y, -x) gives 90 deg Counter-Clockwise
                vec_left = pygame.math.Vector2(vec_in.y, -vec_in.x)

                # Stop Line Point: Junction Center - (Back to edge) + (Left to lane center)
                entry_point = p_junc - (vec_in * junction_half_size) + (vec_left * (lane_width / 2))

                # Store straight approach path
                path_start_real = p_start + (vec_left * (lane_width / 2))
                paths[(start_id, junc_id)] = [path_start_real, entry_point]

                # --- 2. Calculate Exit Geometry & Turns ---
                next_nodes = self.map.graph.get_neighbors(junc_id, previous_node=start_id)

                for end_id in next_nodes:
                    p_end = pygame.math.Vector2(nodes[end_id])

                    # Vector pointing out of the junction
                    vec_out = (p_end - p_junc).normalize()

                    # Output lane offset (Left side relative to exit direction)
                    vec_left_out = pygame.math.Vector2(vec_out.y, -vec_out.x)

                    # Exit Point: Junction Center + (Forward to edge) + (Left to lane center)
                    exit_point = p_junc + (vec_out * junction_half_size) + (vec_left_out * (lane_width / 2))

                    # Determine Turn Type
                    alignment = vec_in.dot(vec_out)

                    # Cross Product to determine Left vs Right
                    # In Pygame (Y-Down):
                    # Cross < 0 usually implies Left Turn (CCW)
                    # Cross > 0 usually implies Right Turn (CW)
                    cross_prod = vec_in.x * vec_out.y - vec_in.y * vec_out.x

                    path_points = []

                    if alignment > 0.9:  # Straight
                        path_points = [entry_point, exit_point]

                    elif alignment < -0.9:  # U-Turn (Just connect them straight for now)
                        path_points = [entry_point, exit_point]

                    else:  # 90-Degree Turn
                        if cross_prod < 0:
                            # --- LEFT TURN (Tight in LHT) ---
                            # Pivot: The corner to the LEFT of the entry
                            pivot = p_junc - (vec_in * junction_half_size) + (vec_left * junction_half_size)
                        else:
                            # --- RIGHT TURN (Wide in LHT) ---
                            # Pivot: The corner to the RIGHT of the entry
                            # Right vector is opposite of Left vector
                            vec_right = -vec_left
                            pivot = p_junc - (vec_in * junction_half_size) + (vec_right * junction_half_size)

                        path_points = self.generate_arc(pivot, entry_point, exit_point)

                    paths[(start_id, junc_id, end_id)] = path_points

        return paths

    def generate_arc(self, center, start, end, num_points=15):
        """
        Generates the shortest arc between start and end points around a pivot center.
        """
        points = []
        radius = (start - center).length()

        start_vec = start - center
        end_vec = end - center

        start_angle = atan2(start_vec.y, start_vec.x)
        end_angle = atan2(end_vec.y, end_vec.x)

        # Calculate the angular difference
        diff = end_angle - start_angle

        # Normalize difference to [-PI, PI]
        # This forces the "Short Way" around the circle, fixing the loop issue.
        while diff <= -pi:
            diff += 2 * pi
        while diff > pi:
            diff -= 2 * pi

        for i in range(num_points + 1):
            t = i / num_points
            current_angle = start_angle + t * diff
            x = center.x + radius * cos(current_angle)
            y = center.y + radius * sin(current_angle)
            points.append(pygame.math.Vector2(x, y))

        return points

    def get_path_for_nodes(self, node_sequence):
        return self.paths.get(tuple(node_sequence))