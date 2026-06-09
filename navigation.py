import time
import threading
import sensorsweep as ss
import astar as ast
from state import robot

def navigation_loop():
    while True:
        with robot.lock:
            if robot.path:
                robot.is_moving = True
                next_x, next_y = robot.path.pop(0)

                if robot.world_map[next_x, next_y] == 1:
                    print("Path blocked, replanning")
                    ss.sensor_sweep(
                        robot.x, robot.y,
                        robot.world_map,
                        robot.robot_map,
                        sensor_range=2
                    )

                    result = ast.a_star(
                        start=(robot.x, robot.y),
                        goal=robot.goal,
                        robot_map=robot.robot_map,
                        grid_size=robot.grid_size
                    )

                    if result is None:
                        print("No path found — stopping")
                        robot.path = []
                    else:
                        robot.path = result[1:]

                    robot.is_moving = False
                else:
                    robot.x, robot.y = next_x, next_y
                    ss.sensor_sweep(
                        robot.x, robot.y,
                        robot.world_map,
                        robot.robot_map,
                        sensor_range=2
                    )

                if not robot.path:
                    robot.is_moving = False
                    print(f"Reached destination ({robot.x}, {robot.y})")

        time.sleep(0.3)

def start():
    thread = threading.Thread(target=navigation_loop, daemon=True)
    thread.start()