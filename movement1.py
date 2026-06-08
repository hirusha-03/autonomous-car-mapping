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

x,y = 0, 0


plt.ion()


fig, ax = plt.subplots(figsize=(6, 6))


ans = ''
while (ans != 'q'):
    


    display_grid = grid.copy()
    #  Draw robot on top of trail
    display_grid[x, y] = 0.5

    ans = input("(w/a/s/d to move, q to quit): ")

    if ans == 'w':
        print("Moving up")
        print(f"before: (x {x}, y {y})")
        y = y + 1
        x = x
        print(f"Current position: (x {x}, y {y})")
    elif ans == 's':
        print("Moving down")
        print(f"before: (x {x}, y {y})")
        y -= 1 
        x = x 
        print(f"Current position: (x {x}, y {y})")
    elif ans == 'a':
        print("Moving left")
        print(f"before: (x {x}, y {y})")
        x -= 1
        y = y
        print(f"Current position: (x {x}, y {y})")
    elif ans == 'd':
        print("Moving right")
        print(f"before: (x {x}, y {y})")
        x += 1
        y = y
        print(f"Current position: (x {x}, y {y})")
    elif ans == 'q':
        print("Quitting")
    else:
        print("Invalid command. Use w/a/s/d to move or q to quit.")





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