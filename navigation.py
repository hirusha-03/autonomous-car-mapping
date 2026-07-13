import time
import threading
import sensorsweep as ss
import astar as ast
import frontier as fr
from state import robot, DIRECTION_VECTORS

MAX_REPLAN_ATTEMPTS = 3

# Calibration — must match robot_firmware.ino constants
TURN_90_MS  = 650   # ms to spin 90 degrees in place
FORWARD_MS  = 800   # ms to drive one grid cell forward (30.48cm / 1ft cell, pending real-robot recalibration)

# Wait for ESP32 to drain the command queue before updating robot position.
# Generous on purpose: each queued command costs multiple full WiFi HTTP
# round-trips (sensor POST, command GET, and a /stop_flag GET on every
# ~150ms motion chunk — see robot_firmware.ino runMotion), so a normal
# turn+forward pair can legitimately take several seconds on a real
# connection. This is a safety net against genuine stalls, not a tight
# timing budget.
COMMAND_DRAIN_TIMEOUT = 12.0  # seconds


def _required_direction(from_pos, to_pos):
    """Return the direction index (0-3) needed to move from from_pos to to_pos,
    or None if the two positions aren't exactly one cardinal step apart (e.g.
    robot.x/y desynced from the path after a command-drain timeout)."""
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    try:
        return DIRECTION_VECTORS.index((dx, dy))
    except ValueError:
        return None


def _enqueue_move(current_dir, required_dir):
    """
    Enqueue turn command(s) + forward command to move one cell.
    Updates robot.direction. Caller must hold robot.lock.
    Returns updated direction.
    """
    diff = (required_dir - current_dir) % 4

    if diff == 1:
        robot.command_queue.append({"cmd": "left",  "duration_ms": TURN_90_MS})
    elif diff == 3:
        robot.command_queue.append({"cmd": "right", "duration_ms": TURN_90_MS})
    elif diff == 2:
        robot.command_queue.append({"cmd": "uturn", "duration_ms": TURN_90_MS * 2})
    # diff == 0 → already facing correct direction, no turn needed

    robot.command_queue.append({"cmd": "forward", "duration_ms": FORWARD_MS})
    return required_dir


def _wait_for_commands_drained():
    """Block until ESP32 has consumed all queued commands (or timeout)."""
    deadline = time.time() + COMMAND_DRAIN_TIMEOUT
    while time.time() < deadline:
        with robot.lock:
            if not robot.command_queue:
                return True
        time.sleep(0.1)
    return False  # timed out


def navigation_loop():
    replan_count = 0

    while True:
        try:
            replan_count = _navigation_tick(replan_count)
        except Exception as e:
            # Anything unexpected here must not kill the loop permanently —
            # a dead nav thread looks exactly like "robot doesn't respond to
            # explore/navigate", with no error surfaced anywhere (that's how
            # a prior ValueError in _required_direction went unnoticed).
            print(f"[NAV] navigation_loop error (continuing): {e!r}")
            with robot.lock:
                robot.path = []
                robot.goal = None
                robot.is_moving = False
                if robot.mode in ("goto", "manual"):
                    robot.mode = "idle"
            replan_count = 0
        time.sleep(0.1)


def _navigation_tick(replan_count):
    """One iteration of the nav loop. Returns the (possibly updated) replan_count."""
    with robot.lock:
        if robot.mode == "manual":
            pass  # /manual/move drives the queue directly, nothing to plan

        elif robot.path:
            robot.is_moving = True
            next_x, next_y = robot.path[0]  # peek, don't pop yet

            # Check for obstacle at next cell using real sensor data
            if robot.sensor_updated:
                # Real sensor: obstacle is detected by /sensor_data endpoint
                # robot_map is already updated by real sensor readings
                obstacle_detected = robot.robot_map[next_x, next_y] >= 1.386
            else:
                # Simulation fallback: check world_map directly
                obstacle_detected = robot.world_map[next_x, next_y] == 1

            if obstacle_detected:
                replan_count += 1
                print(f"Path blocked at ({next_x},{next_y}), replanning ({replan_count}/{MAX_REPLAN_ATTEMPTS})")

                if replan_count >= MAX_REPLAN_ATTEMPTS:
                    print("Max replans reached — abandoning goal")
                    robot.path = []
                    robot.goal = None
                    robot.is_moving = False
                    robot.mode = "idle"
                    replan_count = 0
                else:
                    if not robot.sensor_updated:
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
                # Safe to move — pop the step and enqueue motor commands
                robot.path.pop(0)
                required_dir = _required_direction((robot.x, robot.y), (next_x, next_y))
                if required_dir is None:
                    # robot.x/y is desynced from this path (e.g. after a drain
                    # timeout) — the path is no longer valid, drop it and let
                    # the outer loop replan/re-target from the real position.
                    print(f"Path step ({robot.x},{robot.y})->({next_x},{next_y}) "
                          f"isn't a cardinal step — discarding stale path")
                    robot.path = []
                    robot.goal = None
                    robot.is_moving = False
                else:
                    robot.direction = _enqueue_move(robot.direction, required_dir)
                    robot.pending_move = (next_x, next_y)
                    replan_count = 0

        # --- Autonomous explore mode ---
        elif robot.mode == "explore" and not robot.is_moving:
            target = fr.best_frontier_target(
                robot.robot_map, (robot.x, robot.y), robot.grid_size
            )
            print(f"[EXPLORE] pos=({robot.x},{robot.y}) target={target}")
            if target is None:
                print("Exploration complete — map fully explored")
                robot.mode = "idle"
            else:
                result = ast.a_star(
                    start=(robot.x, robot.y),
                    goal=target,
                    robot_map=robot.robot_map,
                    grid_size=robot.grid_size
                )
                print(f"[EXPLORE] a_star result={result}")
                if result:
                    robot.goal = target
                    robot.path = result[1:]
                else:
                    print(f"Frontier {target} unreachable, skipping")
                    robot.robot_map[target[0], target[1]] = 4.0

    # Wait for ESP32 to drain queued commands before updating position.
    # Gate on pending_move too — the queue may already be empty by the
    # time we check (ESP32 can drain faster than our own poll interval).
    with robot.lock:
        commands_pending = bool(robot.command_queue) or robot.pending_move is not None

    if commands_pending:
        drained = _wait_for_commands_drained()
        if drained:
            with robot.lock:
                if robot.pending_move is not None:
                    robot.x, robot.y = robot.pending_move
                    robot.pending_move = None
                if not robot.path:
                    robot.is_moving = False
                    if robot.goal:
                        print(f"Reached goal ({robot.x}, {robot.y})")
                    robot.goal = None
                    if robot.mode in ("goto", "manual"):
                        robot.mode = "idle"
                # Update map with real sensor now that position moved
                if not robot.sensor_updated:
                    ss.sensor_sweep(robot.x, robot.y,
                                    robot.world_map, robot.robot_map, 2)
        else:
            print("WARNING: command drain timed out — robot may have stalled")
            with robot.lock:
                robot.command_queue.clear()
                robot.pending_move = None
                robot.is_moving = False
                # Path/goal were planned against a position we can no longer
                # trust (we don't know how far the robot actually got before
                # stalling) — drop them so explore/goto replans from scratch.
                robot.path = []
                robot.goal = None
                if robot.mode in ("goto", "manual"):
                    robot.mode = "idle"
    else:
        with robot.lock:
            if not robot.path:
                robot.is_moving = False

    return replan_count


def start():
    thread = threading.Thread(target=navigation_loop, daemon=True)
    thread.start()
