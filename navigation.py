import time
import threading
import sensorsweep as ss
import astar as ast
import frontier as fr
from state import robot

MAX_REPLAN_ATTEMPTS = 3

def navigation_loop():
    replan_count = 0

    while True:
        with robot.lock:
            if robot.path:
                robot.is_moving = True
                next_x, next_y = robot.path.pop(0)

                if robot.world_map[next_x, next_y] == 1:
                    replan_count += 1
                    print(f"Path blocked, replanning ({replan_count}/{MAX_REPLAN_ATTEMPTS})")

                    if replan_count >= MAX_REPLAN_ATTEMPTS:
                        print("Max replans reached — abandoning goal")
                        robot.path = []
                        robot.goal = None
                        robot.is_moving = False
                        replan_count = 0
                    else:
                        ss.sensor_sweep(robot.x, robot.y,
                                        robot.world_map, robot.robot_map, 2)
                        result = ast.a_star(
                            start=(robot.x, robot.y),
                            goal=robot.goal,
                            robot_map=robot.robot_map,
                            grid_size=robot.grid_size
                        )
                        robot.path = result[1:] if result else []
                        if not robot.path:
                            robot.goal = None
                            robot.is_moving = False
                else:
                    robot.x, robot.y = next_x, next_y
                    replan_count = 0
                    ss.sensor_sweep(robot.x, robot.y,
                                    robot.world_map, robot.robot_map, 2)

                if not robot.path:
                    robot.is_moving = False
                    if robot.goal:
                        print(f"Reached ({robot.x}, {robot.y})")
                    robot.goal = None

            # --- Autonomous explore mode ---
            elif robot.exploring and not robot.is_moving:
                target = fr.best_frontier_target(
                    robot.robot_map, (robot.x, robot.y), robot.grid_size
                )
                if target is None:
                    print("Exploration complete — map fully explored")
                    robot.exploring = False
                else:
                    result = ast.a_star(
                        start=(robot.x, robot.y),
                        goal=target,
                        robot_map=robot.robot_map,
                        grid_size=robot.grid_size
                    )
                    if result:
                        robot.goal = target
                        robot.path = result[1:]
                    else:
                        # Can't reach this frontier — mark it and try again next tick
                        print(f"Frontier {target} unreachable, skipping")
                        robot.robot_map[target[0], target[1]] = 4.0  # treat as known

        time.sleep(0.3)


def start():
    thread = threading.Thread(target=navigation_loop, daemon=True)
    thread.start()