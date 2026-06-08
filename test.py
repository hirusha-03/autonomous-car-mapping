

import numpy as np
import matplotlib.pyplot as plt

# Grid size
grid_size = (10, 10)

# Create world
grid = np.zeros(grid_size)

# Obstacles
grid[2:5, 5:7] = 1

# Stores visited cells
#history_grid = grid.copy()

plt.ion()

fig, ax = plt.subplots(figsize=(6, 6))

for x in range(grid_size[0]):

    # Snake pattern
    if x % 2 == 0:
        y_range = range(grid_size[1])
    else:
        y_range = range(grid_size[1] - 1, -1, -1)

    for y in y_range:

        # Skip obstacles
        if grid[x, y] == 1:
            continue

        # Mark current location as visited
        #history_grid[x, y] = 0.8

        # Create display frame
        #display_grid = history_grid.copy()
        display_grid = grid.copy()

        # Draw robot on top of trail
        display_grid[x, y] = 0.5

        ax.clear()

        ax.imshow(
            display_grid.T,
            cmap='gray_r',
            origin='lower',
            vmin=0,
            vmax=1
        )

        ax.set_title(f"Robot Position: ({x}, {y})")
        ax.set_xticks(range(grid_size[0]))
        ax.set_yticks(range(grid_size[1]))
        ax.grid(True)

        plt.pause(1)

plt.ioff()
plt.show()