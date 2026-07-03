import numpy as np
import random
from state import DIRECTION_VECTORS

SENSOR_NOISE = 0.1       # simulation only — 10% noise rate
LOG_ODD_HIT  =  2.2      # log-odds added when obstacle detected
LOG_ODD_MISS = -2.2      # log-odds added when cell is free
LOG_ODD_CLAMP = 4.0      # max |log-odds| to prevent infinite confidence

# Physical calibration
CELL_SIZE_CM   = 30.0    # one grid cell = this many cm (tune on real robot)
OBSTACLE_RATIO = 0.8     # distance < CELL_SIZE * ratio → treat as obstacle


def update_from_real_sensor(robot_x, robot_y, direction, distance_cm, robot_map):
    """
    Update robot_map using a single front-facing HC-SR04 reading.
    Marks the cell directly in front as occupied or free.
    All cells between the robot and the detected obstacle are marked free.
    """
    dx, dy = DIRECTION_VECTORS[direction]
    rows, cols = robot_map.shape
    obstacle_cell = int(distance_cm / CELL_SIZE_CM)  # how many cells away
    obstacle_cell = max(1, obstacle_cell)             # at least 1 cell ahead

    # Mark cells along the ray
    for step in range(1, obstacle_cell + 1):
        cx = robot_x + dx * step
        cy = robot_y + dy * step
        if not (0 <= cx < rows and 0 <= cy < cols):
            break

        if step < obstacle_cell or distance_cm >= CELL_SIZE_CM * OBSTACLE_RATIO * obstacle_cell:
            # Cell is in front of the obstacle (or beyond sensor range) → free
            robot_map[cx, cy] = np.clip(
                robot_map[cx, cy] + LOG_ODD_MISS, -LOG_ODD_CLAMP, LOG_ODD_CLAMP
            )
        else:
            # This is the obstacle cell
            robot_map[cx, cy] = np.clip(
                robot_map[cx, cy] + LOG_ODD_HIT, -LOG_ODD_CLAMP, LOG_ODD_CLAMP
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
