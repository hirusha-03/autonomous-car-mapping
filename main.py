import csv
import os
from datetime import datetime, timezone

from fastapi import FastAPI
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from state import robot, DIRECTION_NAMES, DIRECTION_VECTORS
import sensorsweep as ss
import astar as ast
import frontier as fr
from navigation import start as start_navigation, TURN_90_MS, FORWARD_MS

CALIB_LOG_PATH = os.path.join(os.path.dirname(__file__), "calibration_log.csv")
CALIB_LOG_HEADER = ["timestamp", "commanded_deg", "gyro_deg", "measured_deg"]

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
    distance_left_cm: float | None = None
    distance_right_cm: float | None = None
    accel_x: float | None = None
    accel_y: float | None = None
    accel_z: float | None = None
    gyro_z: float | None = None


class ManualMove(BaseModel):
    cmd: str  # "forward" | "left" | "right" | "stop"


class SpeedSetting(BaseModel):
    left_pct: int
    right_pct: int


class CalibrateTurn(BaseModel):
    duration_ms: int
    dir: str  # "left" | "right"


class CalibReport(BaseModel):
    commanded_deg: float
    gyro_deg: float


class CalibMeasured(BaseModel):
    measured_deg: float


# ── ESP32 endpoints ───────────────────────────────────────────────────────────

@app.post("/sensor_data")
def receive_sensor_data(data: SensorData):
    """ESP32 pushes HC-SR04 readings here every ~100 ms (front required, left/right optional)."""
    with robot.lock:
        robot.sensor_distance_cm = data.distance_cm
        robot.sensor_updated = True
        ss.update_from_real_sensor(
            robot.x, robot.y, robot.direction,
            data.distance_cm, robot.robot_map
        )
        if data.distance_left_cm is not None:
            robot.sensor_distance_left_cm = data.distance_left_cm
            ss.update_from_real_sensor(
                robot.x, robot.y, robot.direction,
                data.distance_left_cm, robot.robot_map, side="left"
            )
        if data.distance_right_cm is not None:
            robot.sensor_distance_right_cm = data.distance_right_cm
            ss.update_from_real_sensor(
                robot.x, robot.y, robot.direction,
                data.distance_right_cm, robot.robot_map, side="right"
            )
        if data.accel_x is not None:
            robot.accel_x = data.accel_x
        if data.accel_y is not None:
            robot.accel_y = data.accel_y
        if data.accel_z is not None:
            robot.accel_z = data.accel_z
        if data.gyro_z is not None:
            robot.gyro_z = data.gyro_z
    return {"status": "ok"}


@app.get("/command")
def get_command():
    """ESP32 polls here to receive the next motor command (FIFO)."""
    with robot.lock:
        cmd = robot.command_queue.popleft() if robot.command_queue else {"cmd": "none", "duration_ms": 0}
        cmd["left_pct"] = robot.motor_speed_left_pct
        cmd["right_pct"] = robot.motor_speed_right_pct
    return cmd


@app.post("/speed")
def set_speed(setting: SpeedSetting):
    """Sets per-side motor PWM duty (%), applied by the ESP32 on its next
    /command poll. Independent left/right (rather than one speed + a fixed
    trim) because measured drift direction wasn't consistent across test
    runs — see ai_context/INDEX.md. Clamped to 10-100: too low can't overcome
    motor static friction and stalls without moving."""
    left = max(10, min(100, setting.left_pct))
    right = max(10, min(100, setting.right_pct))
    with robot.lock:
        robot.motor_speed_left_pct = left
        robot.motor_speed_right_pct = right
    return {"status": "ok", "left_pct": left, "right_pct": right}


@app.get("/stop_flag")
def get_stop_flag():
    """ESP32 polls this mid-motion (between chunked delay steps) to abort
    a drive/turn early on manual stop. Consume-once: clears after being read."""
    with robot.lock:
        stop = robot.stop_requested
        robot.stop_requested = False
    return {"stop": stop}


@app.post("/obstacle_stop")
def obstacle_stop():
    """ESP32 calls this when it self-aborted a forward drive because the front
    sensor tripped the safety-stop distance mid-motion (see runMotion's live
    obstacle check in robot_firmware.ino). The queued forward command did not
    complete, so the robot's position did not actually advance — don't apply
    pending_move. Mark the target cell as an obstacle so replanning avoids it."""
    with robot.lock:
        target = robot.pending_move
        robot.pending_move = None
        robot.command_queue.clear()
        robot.is_moving = False
        robot.path = []
        robot.goal = None
        if robot.mode in ("goto", "manual"):
            robot.mode = "idle"
        # explore mode is left as-is: the nav loop's idle-tick branch will
        # pick a new frontier target on its own now that the map reflects
        # the obstacle, so no extra handling is needed here.

        if target is not None:
            tx, ty = target
            robot.robot_map[tx, ty] = max(
                -ss.LOG_ODD_CLAMP, min(ss.LOG_ODD_CLAMP, robot.robot_map[tx, ty] + ss.LOG_ODD_HIT)
            )
    return {"status": "ok"}


# ── Dashboard / frontend endpoints ───────────────────────────────────────────

@app.on_event("startup")
def startup():
    # Seed the map with one simulated sweep so the UI isn't blank
    ss.sensor_sweep(robot.x, robot.y, robot.world_map, robot.robot_map, sensor_range=2)
    start_navigation()

    if os.path.exists(CALIB_LOG_PATH):
        with open(CALIB_LOG_PATH, newline="") as f:
            robot.calib_log_count = sum(1 for _ in csv.reader(f)) - 1  # minus header row


@app.post("/map/reset")
def reset_map():
    """User-triggered full reset: clears the map, drive state, and command
    queue, and puts the robot back at grid center. Does not touch sensor
    calibration (motor PWM %) since that's a robot property, not a map one."""
    with robot.lock:
        robot.reset()
        ss.sensor_sweep(robot.x, robot.y, robot.world_map, robot.robot_map, sensor_range=2)
    return {"status": "ok", "message": "Map and state reset"}


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
            "sensor_distance_left_cm": robot.sensor_distance_left_cm,
            "sensor_distance_right_cm": robot.sensor_distance_right_cm,
            "sensor_connected": robot.sensor_updated,
            "accel_x": robot.accel_x,
            "accel_y": robot.accel_y,
            "accel_z": robot.accel_z,
            "gyro_z": robot.gyro_z,
            "pending_commands": len(robot.command_queue),
            "motor_speed_left_pct": robot.motor_speed_left_pct,
            "motor_speed_right_pct": robot.motor_speed_right_pct,
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
        # Stop must always go through, even mid-motion — it's the one command
        # allowed to interrupt is_moving instead of being blocked by it.
        if move.cmd == "stop":
            robot.mode = "idle"
            robot.path = []
            robot.goal = None
            robot.pending_move = None
            robot.command_queue.clear()
            robot.is_moving = False
            robot.stop_requested = True
            return {"status": "ok", "message": "Stopped"}

        if move.cmd not in ("forward", "reverse", "left", "right"):
            return {"status": "error", "message": f"Unknown command '{move.cmd}'"}

        # A new manual command interrupts whatever's currently running (goto,
        # explore, or a previous manual move) instead of being rejected — lets
        # the user chain arrow-key presses without an explicit stop in between.
        if robot.is_moving:
            robot.pending_move = None
            robot.command_queue.clear()
            robot.stop_requested = True

        robot.mode = "manual"
        robot.path = []
        robot.goal = None
        robot.is_moving = False

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

        # forward / reverse — reverse targets the cell behind the robot
        dx, dy = DIRECTION_VECTORS[robot.direction]
        if move.cmd == "reverse":
            dx, dy = -dx, -dy
        nx, ny = robot.x + dx, robot.y + dy
        if not (0 <= nx < robot.grid_size[0] and 0 <= ny < robot.grid_size[1]):
            robot.mode = "idle"
            return {"status": "error", "message": "Blocked by grid boundary"}
        if robot.robot_map[nx, ny] >= 1.386:
            robot.mode = "idle"
            return {"status": "error", "message": "Blocked by obstacle"}

        if move.cmd == "forward":
            robot.command_queue.append({"cmd": "forward", "duration_ms": FORWARD_MS})
        else:
            robot.command_queue.append({"cmd": "reverse", "duration_ms": FORWARD_MS})
        robot.pending_move = (nx, ny)
        robot.is_moving = True
        return {"status": "ok", "moving_to": (nx, ny)}


@app.post("/calibrate/turn")
def calibrate_turn(turn: CalibrateTurn):
    """Debug/calibration probe: spins the robot for an arbitrary duration_ms
    (bypassing TURN_90_MS) so the real ms-per-degree can be measured by hand
    (protractor/tape) and bisected without reflashing firmware each guess.
    Doesn't touch robot.direction/position — this is an off-grid test spin,
    not a tracked navigation move."""
    if turn.dir not in ("left", "right"):
        return {"status": "error", "message": f"Unknown direction '{turn.dir}'"}
    if turn.duration_ms <= 0:
        return {"status": "error", "message": "duration_ms must be positive"}

    with robot.lock:
        if robot.is_moving:
            return {"status": "error", "message": "Robot is already moving"}
        robot.command_queue.append({"cmd": f"calib_{turn.dir}", "duration_ms": turn.duration_ms})
        robot.is_moving = True
        robot.mode = "manual"
    return {"status": "ok", "duration_ms": turn.duration_ms, "dir": turn.dir}


@app.post("/calibrate/report")
def calibrate_report(report: CalibReport):
    """ESP32 posts here right after a gyro-corrected turn (see turnByAngle in
    robot_firmware.ino) completes, reporting what the gyro measured. Overwrites
    any previous pending report — only the most recent unlogged turn can be
    measured/submitted at a time."""
    with robot.lock:
        robot.pending_calib_report = {
            "commanded_deg": report.commanded_deg,
            "gyro_deg": report.gyro_deg,
        }
    return {"status": "ok"}


@app.get("/calibrate/pending")
def calibrate_pending():
    """Dashboard polls this to know what to go measure with a protractor."""
    with robot.lock:
        count = robot.calib_log_count
        if robot.pending_calib_report is None:
            return {"pending": False, "logged_count": count}
        return {"pending": True, "logged_count": count, **robot.pending_calib_report}


@app.post("/calibrate/measured")
def calibrate_measured(measured: CalibMeasured):
    """User submits their real protractor reading for the currently-pending
    gyro report, appending one row to calibration_log.csv for later fitting
    (see calibrate_fit.py)."""
    with robot.lock:
        if robot.pending_calib_report is None:
            return {"status": "error", "message": "No pending measurement to log"}

        row = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "commanded_deg": robot.pending_calib_report["commanded_deg"],
            "gyro_deg": robot.pending_calib_report["gyro_deg"],
            "measured_deg": measured.measured_deg,
        }
        write_header = not os.path.exists(CALIB_LOG_PATH)
        with open(CALIB_LOG_PATH, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=CALIB_LOG_HEADER)
            if write_header:
                writer.writeheader()
            writer.writerow(row)

        robot.pending_calib_report = None
        robot.calib_log_count += 1
        count = robot.calib_log_count
    return {"status": "ok", "logged_count": count}
