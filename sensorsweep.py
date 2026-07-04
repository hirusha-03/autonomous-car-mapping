import numpy as np
import random
from state import DIRECTION_VECTORS

SENSOR_NOISE = 0.1       # simulation only — 10% noise rate
LOG_ODD_HIT  =  2.2      # log-odds added when obstacle detected
LOG_ODD_MISS = -2.2      # log-odds added when cell is free
LOG_ODD_CLAMP = 4.0      # max |log-odds| to prevent infinite confidence

# Physical calibration
CELL_SIZE_CM   = 30.0    # one grid cell = this many cm (tune on real robot)

# Must match robot_firmware.ino's readDistanceOn() timeout fallback — that's
# the sentinel it returns when pulseIn() times out (nothing within ~4.3m
# range), i.e. the only reliable signal that no real echo came back.
NO_ECHO_CM = 400.0


# Side sensors are mounted at a fixed ~30deg angle off the front sensor, too
# narrow to treat as a separate cardinal ray on a 4-directional grid. Approximated
# as a diagonal ray (front vector + perpendicular vector) — mostly-forward with a
# sideways component, which is what a shallow mounting angle actually covers.
def _diagonal_vector(direction, side):
    dx, dy = DIRECTION_VECTORS[direction]
    perp_index = (direction + 1) % 4 if side == "left" else (direction - 1) % 4
    pdx, pdy = DIRECTION_VECTORS[perp_index]
    return dx + pdx, dy + pdy


def update_from_real_sensor(robot_x, robot_y, direction, distance_cm, robot_map, side=None):
    """
    Update robot_map from one HC-SR04 reading (front, or angled left/right).
    Marks the cell(s) along that ray as occupied or free.
    `side=None` → straight-ahead front ray; `side="left"|"right"` → diagonal
    ray approximating the angled side sensor.
    """
    if side is None:
        dx, dy = DIRECTION_VECTORS[direction]
    else:
        dx, dy = _diagonal_vector(direction, side)
    rows, cols = robot_map.shape
    obstacle_cell = int(distance_cm / CELL_SIZE_CM)  # how many cells away
    obstacle_cell = max(1, obstacle_cell)             # at least 1 cell ahead
    got_echo = distance_cm < NO_ECHO_CM               # False = sensor timed out, nothing in range

    # Mark cells along the ray
    for step in range(1, obstacle_cell + 1):
        cx = robot_x + dx * step
        cy = robot_y + dy * step
        if not (0 <= cx < rows and 0 <= cy < cols):
            break

        if step == obstacle_cell and got_echo:
            # Ray's endpoint, and a real echo came back here — the obstacle
            robot_map[cx, cy] = np.clip(
                robot_map[cx, cy] + LOG_ODD_HIT, -LOG_ODD_CLAMP, LOG_ODD_CLAMP
            )
        else:
            # Cell is in front of the obstacle, or the sensor timed out
            # (nothing detected within range) → free
            robot_map[cx, cy] = np.clip(
                robot_map[cx, cy] + LOG_ODD_MISS, -LOG_ODD_CLAMP, LOG_ODD_CLAMP
            )


# ── Simulation fallback (used when real sensor not connected) ─────────────────

def sensor_sweep(robot_x, robot_y, world_map, robot_map, sensor_range):
    """Simulated multi-direction sweep from world_map with noise."""
    for dx in range(-sensor_range, sensor_range + 1):
        for dy in range(-sensor_range, sensor_range + 1):
            if abs(dx) + abs(dy) > sensor_range:
                continue
            cell_x = robot_x + dx
            cell_y = robot_y + dy
            if not (0 <= cell_x < len(world_map) and 0 <= cell_y < len(world_map[0])):
                continue

            actual = world_map[cell_x][cell_y]
            if random.random() < SENSOR_NOISE:
                actual = 1 - actual

            delta = LOG_ODD_HIT if actual == 1 else LOG_ODD_MISS
            robot_map[cell_x][cell_y] = np.clip(
                robot_map[cell_x][cell_y] + delta, -LOG_ODD_CLAMP, LOG_ODD_CLAMP
            )


def sigmoid(x):
    return 1 / (1 + np.exp(-x))
