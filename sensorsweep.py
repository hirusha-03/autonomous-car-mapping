import numpy as np
import random

SENSOR_NOISE = 0.1  # 10% chance sensor is wrong

def sensor_sweep(robot_x, robot_y, world_map, robot_map, sensor_range):

    for dx in range(-sensor_range, sensor_range + 1):
        for dy in range(-sensor_range, sensor_range + 1):

            manhattan_distance = abs(dx) + abs(dy)

            if manhattan_distance <= sensor_range:
                cell_x = robot_x + dx
                cell_y = robot_y + dy

                if 0 <= cell_x < len(world_map) and 0 <= cell_y < len(world_map[0]):
                    
                    # Read actual world value
                    actual = world_map[cell_x][cell_y]

                    # Simulate sensor noise — 10% chance of wrong reading
                    if random.random() < SENSOR_NOISE:
                        actual = 1 - actual  # flip the reading

                    if actual == 1:
                        robot_map[cell_x][cell_y] += 2.2
                    else:
                        robot_map[cell_x][cell_y] -= 2.2

                    # Clamp to prevent infinite confidence
                    robot_map[cell_x][cell_y] = np.clip(
                        robot_map[cell_x][cell_y], -4.0, 4.0
                    )

def sigmoid(x):
    return 1 / (1 + np.exp(-x))