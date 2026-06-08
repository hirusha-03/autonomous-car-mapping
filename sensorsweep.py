def sensor_sweep(robot_x,robot_y, world_map, robot_map,sensor_range):
    

    # Loop through the sensor range
    for dx in range(-sensor_range, sensor_range + 1):
        for dy in range(-sensor_range, sensor_range + 1):

            manhattan_distance = abs(dx) + abs(dy)
            # Only sense cells within the sensor range
            if manhattan_distance <= sensor_range:
                # Calculate the absolute position of the cell being sensed
                cell_x = robot_x + dx
                cell_y = robot_y + dy

                # Check if the cell is within the bounds of the world map
                if 0 <= cell_x < len(world_map) and 0 <= cell_y < len(world_map[0]):
                    # Update the robot's map with the sensed value
                    if world_map[cell_x][cell_y] == 1:
                        robot_map[cell_x][cell_y] = 0.9  # Mark as obstacle
                    else:
                        robot_map[cell_x][cell_y] = 0.1  # Mark as free space