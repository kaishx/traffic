import pygame
import math

# Detection parameters (trapezium) - scaled down by 50%
CAR_LENGTH = 20.0  # visual car length used for spacing decisions
NEAR_BASE_DIST = 0.25 * CAR_LENGTH  # half of previous near-base distance
DETECTION_HEIGHT = 2.0 * CAR_LENGTH  # half of previous trapezium height
NEAR_BASE_WIDTH = 0.5 * CAR_LENGTH
FAR_BASE_WIDTH = 1.5 * CAR_LENGTH


def _in_detection_trapezium(heading: 'pygame.math.Vector2', rel: 'pygame.math.Vector2') -> bool:
    """Return True if the relative vector `rel` (from car to object) lies inside
    the forward trapezium defined by NEAR_BASE_DIST, DETECTION_HEIGHT,
    NEAR_BASE_WIDTH and FAR_BASE_WIDTH.
    """
    forward = rel.dot(heading)
    if forward < NEAR_BASE_DIST or forward > (NEAR_BASE_DIST + DETECTION_HEIGHT):
        return False
    # lateral distance perpendicular to heading
    lateral_vec = rel - heading * forward
    lateral = lateral_vec.length()
    # Interpolate half-width between near and far bases
    near_half = NEAR_BASE_WIDTH / 2.0
    far_half = FAR_BASE_WIDTH / 2.0
    t = (forward - NEAR_BASE_DIST) / DETECTION_HEIGHT
    half_width = near_half + (far_half - near_half) * t
    return lateral <= half_width


def update_car_waiting_states(sim_map, cars, lookahead_factor: float = 0.4, min_lookahead: float = 20.0):
    """Check for obstacles in front of cars and set them to WAITING or resume them.

    Only consider obstacles that are within the trapezium in front of the car
    as defined by the constants above.
    """
    if not cars:
        return

    for car in cars:
        # Ignore cars with no meaningful path or that have finished their path
        if not getattr(car, 'node_path', None) or getattr(car, 'path_index', 0) >= max(0, len(car.node_path) - 1):
            # If the car was waiting but has no path, resume to avoid permanent blockage
            if getattr(car, 'state', None) and getattr(car.state, 'name', '') == 'WAITING':
                try:
                    car.resume()
                except Exception:
                    pass
            continue

        # Ignore stopped cars and cars currently turning
        if getattr(car, 'speed', 0) == 0:
            continue
        if getattr(car, 'state', None) is not None and getattr(car.state, 'name', '') == 'TURNING':
            continue

        heading = pygame.math.Vector2(math.cos(car.angle), math.sin(car.angle))
        pos = pygame.math.Vector2(car.x, car.y)

        blocked = False

        # Check other cars in the detection trapezium in front of the car
        for other in cars:
            if other is car:
                continue
            rel = pygame.math.Vector2(other.x, other.y) - pos
            if _in_detection_trapezium(heading, rel):
                blocked = True
                break

        # If not blocked by cars, check gates in front (also use trapezium)
        if not blocked:
            for (node, nbr), state in sim_map.gates.items():
                if state != 'UP':
                    continue
                node_pos = pygame.math.Vector2(sim_map.nodes[node])
                nbr_pos = pygame.math.Vector2(sim_map.nodes[nbr])
                dir_vec = nbr_pos - node_pos
                if dir_vec.length() == 0:
                    continue
                unit = dir_vec.normalize()
                # gate position slightly outside the intersection (matches draw logic)
                gate_pos = node_pos + unit * (sim_map.LANE_WIDTH + 4)
                rel2 = gate_pos - pos
                if _in_detection_trapezium(heading, rel2):
                    blocked = True
                    break

        # Apply state changes: call wait() if blocked, otherwise resume any waiting car
        try:
            if blocked:
                car.wait()
            else:
                if getattr(car, 'state', None) is not None and getattr(car.state, 'name', '') == 'WAITING':
                    car.resume()
        except Exception:
            # Be defensive in case objects passed in aren't full car objects
            pass

