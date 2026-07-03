from fastapi import FastAPI
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from state import robot, DIRECTION_NAMES, DIRECTION_VECTORS
import sensorsweep as ss
import astar as ast
import frontier as fr
from navigation import start as start_navigation, TURN_90_MS, FORWARD_MS

app = FastAPI(
    title="Autonomous Car API",
    description="ESP32 ↔ server endpoints and dashboard control for the autonomous car.",
    version="1.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)


class Target(BaseModel):
    x: int
    y: int


class SensorData(BaseModel):
    distance_cm: float


class ManualMove(BaseModel):
    cmd: str  # "forward" | "left" | "right" | "stop"


# ── ESP32 endpoints ───────────────────────────────────────────────────────────

@app.post("/sensor_data")
def receive_sensor_data(data: SensorData):
    """ESP32 pushes front HC-SR04 reading here every ~100 ms."""
    with robot.lock:
        robot.sensor_distance_cm = data.distance_cm
        robot.sensor_updated = True
        ss.update_from_real_sensor(
            robot.x, robot.y, robot.direction,
            data.distance_cm, robot.robot_map
        )
    return {"status": "ok"}


@app.get("/command")
def get_command():
    """ESP32 polls here to receive the next motor command (FIFO)."""
    with robot.lock:
        if robot.command_queue:
            return robot.command_queue.popleft()
    return {"cmd": "none", "duration_ms": 0}


@app.get("/stop_flag")
def get_stop_flag():
    """ESP32 polls this mid-motion (between chunked delay steps) to abort
    a drive/turn early on manual stop. Consume-once: clears after being read."""
    with robot.lock:
        stop = robot.stop_requested
        robot.stop_requested = False
    return {"stop": stop}


# ── Dashboard / frontend endpoints ───────────────────────────────────────────

@app.on_event("startup")
def startup():
    # Seed the map with one simulated sweep so the UI isn't blank
    ss.sensor_sweep(robot.x, robot.y, robot.world_map, robot.robot_map, sensor_range=2)
    start_navigation()


@app.get("/map")
def get_map():
    with robot.lock:
        return {
            "robot_position": (robot.x, robot.y),
            "robot_direction": DIRECTION_NAMES[robot.direction],
            "robot_map": robot.robot_map.tolist(),
            "world_map": robot.world_map.tolist(),
            "path": robot.path,
            "is_moving": robot.is_moving,
            "mode": robot.mode,
            "exploring": robot.mode == "explore",  # kept for frontend compat
            "frontiers": fr.find_frontiers(robot.robot_map, robot.grid_size),
            "sensor_distance_cm": robot.sensor_distance_cm,
            "sensor_connected": robot.sensor_updated,
            "pending_commands": len(robot.command_queue),
        }


@app.post("/navigate")
def navigate(target: Target):
    with robot.lock:
        if not (0 <= target.x < robot.grid_size[0] and 0 <= target.y < robot.grid_size[1]):
            return {"status": "error", "message": "Target out of bounds"}
        if robot.robot_map[target.x, target.y] >= 1.386:
            return {"status": "error", "message": "Target is an obstacle"}
        if robot.is_moving:
            return {"status": "error", "message": "Robot is already moving"}

        robot.goal = (target.x, target.y)
        result = ast.a_star(
            start=(robot.x, robot.y),
            goal=robot.goal,
            robot_map=robot.robot_map,
            grid_size=robot.grid_size
        )
        if result is None:
            return {"status": "error", "message": "No path found"}

        robot.mode = "goto"
        robot.path = result[1:]
        return {"status": "ok", "path": robot.path}


@app.get("/status")
def get_status():
    with robot.lock:
        return {
            "is_moving": robot.is_moving,
            "x": robot.x,
            "y": robot.y,
            "direction": DIRECTION_NAMES[robot.direction],
            "goal": robot.goal,
            "sensor_distance_cm": robot.sensor_distance_cm,
            "mode": robot.mode,
        }


@app.post("/explore/start")
def start_explore():
    with robot.lock:
        if robot.is_moving:
            return {"status": "error", "message": "Robot is moving"}
        robot.mode = "explore"
        robot.goal = None
        robot.path = []
        return {"status": "ok", "message": "Autonomous exploration started"}


@app.post("/explore/stop")
def stop_explore():
    with robot.lock:
        robot.mode = "idle"
        robot.path = []
        robot.goal = None
        robot.is_moving = False
        robot.pending_move = None
        robot.command_queue.clear()
        return {"status": "ok", "message": "Exploration stopped"}


@app.post("/manual/move")
def manual_move(move: ManualMove):
    """Device/user-driven control: joystick or arrow-key input from the dashboard."""
    with robot.lock:
        if robot.is_moving:
            return {"status": "error", "message": "Robot is already moving"}

        if move.cmd == "stop":
            robot.mode = "idle"
            robot.path = []
            robot.goal = None
            robot.pending_move = None
            robot.command_queue.clear()
            robot.stop_requested = True
            return {"status": "ok", "message": "Stopped"}

        if move.cmd not in ("forward", "left", "right"):
            return {"status": "error", "message": f"Unknown command '{move.cmd}'"}

        # Any in-progress goto/explore is superseded by manual control
        robot.mode = "manual"
        robot.path = []
        robot.goal = None
        robot.stop_requested = False

        if move.cmd == "left":
            robot.direction = (robot.direction + 1) % 4
            robot.command_queue.append({"cmd": "left", "duration_ms": TURN_90_MS})
            robot.is_moving = True
            return {"status": "ok", "direction": DIRECTION_NAMES[robot.direction]}

        if move.cmd == "right":
            robot.direction = (robot.direction - 1) % 4
            robot.command_queue.append({"cmd": "right", "duration_ms": TURN_90_MS})
            robot.is_moving = True
            return {"status": "ok", "direction": DIRECTION_NAMES[robot.direction]}

        # forward
        dx, dy = DIRECTION_VECTORS[robot.direction]
        nx, ny = robot.x + dx, robot.y + dy
        if not (0 <= nx < robot.grid_size[0] and 0 <= ny < robot.grid_size[1]):
            robot.mode = "idle"
            return {"status": "error", "message": "Blocked by grid boundary"}
        if robot.robot_map[nx, ny] >= 1.386:
            robot.mode = "idle"
            return {"status": "error", "message": "Blocked by obstacle"}

        robot.command_queue.append({"cmd": "forward", "duration_ms": FORWARD_MS})
        robot.pending_move = (nx, ny)
        robot.is_moving = True
        return {"status": "ok", "moving_to": (nx, ny)}
