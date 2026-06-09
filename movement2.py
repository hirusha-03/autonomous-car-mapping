import numpy as np
import matplotlib.pyplot as plt
import sensorsweep as ss
import astar as ast
import time

# Grid size
grid_size = (10, 10)

# World map
grid = np.zeros(grid_size)
grid[2:5, 5:7] = 1
world_map = grid.copy()

# Robot belief map — initialized to 0.0 (log-odds for 50% uncertainty)
robot_map = np.full(grid_size, 0.0)

# Robot state
x, y = 0, 0
path = []
is_moving = False
goal = None

# Initial scan
ss.sensor_sweep(x, y, world_map, robot_map, sensor_range=2)

# Setup plot
plt.ion()
fig, ax = plt.subplots(figsize=(6, 6))

def on_click(event):
    global path, is_moving, goal

    if event.xdata is None or event.ydata is None:
        return

    if is_moving:
        print("Robot is moving, please wait")
        return

    target_x = int(round(event.xdata))
    target_y = int(round(event.ydata))

    if not (0 <= target_x < grid_size[0] and 0 <= target_y < grid_size[1]):
        print("Clicked outside grid")
        return

    # Threshold now in log-odds space
    if robot_map[target_x, target_y] >= 1.386:
        print("Target is an obstacle")
        return

    goal = (target_x, target_y)
    print(f"Target set to ({target_x}, {target_y})")

    result = ast.a_star(
        start=(x, y),
        goal=goal,
        robot_map=robot_map,
        grid_size=grid_size
    )

    if result is None:
        print("No path found")
        path = []
    else:
        print(f"Path found: {result}")
        path = result[1:]

fig.canvas.mpl_connect('button_press_event', on_click)

def draw(robot_x, robot_y):
    # Convert log-odds to probability for display
    prob_map = ss.sigmoid(robot_map)

    display_map = prob_map.copy()
    display_map[robot_x, robot_y] = 0.25

    for (px, py) in path:
        display_map[px, py] = 0.4

    ax.clear()
    ax.imshow(
        display_map.T,
        cmap='gray_r',
        origin='lower',
        vmin=0,
        vmax=1
    )
    ax.set_title(f"Robot Position: ({robot_x}, {robot_y})")
    ax.set_xticks(range(grid_size[0]))
    ax.set_yticks(range(grid_size[1]))
    ax.grid(True)
    plt.pause(0.1)    

try:
    while True:
        if path:
            is_moving = True
            next_x, next_y = path.pop(0)

            if world_map[next_x, next_y] == 1:
                print("Path blocked, replanning")
                ss.sensor_sweep(x, y, world_map, robot_map, sensor_range=2)

                result = ast.a_star(
                    start=(x, y),
                    goal=goal,
                    robot_map=robot_map,
                    grid_size=grid_size
                )

                if result is None:
                    print("No path found — stopping")
                    path = []
                else:
                    path = result[1:]

                is_moving = False
            else:
                x, y = next_x, next_y
                ss.sensor_sweep(x, y, world_map, robot_map, sensor_range=2)
                draw(x, y)
                time.sleep(0.3)

            if not path:
                is_moving = False
                print(f"Reached destination ({x}, {y})")
        else:
            draw(x, y)

except KeyboardInterrupt:
    print("Simulation stopped")
    plt.ioff()
    plt.close()