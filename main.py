import asyncio
import csv
import os
from datetime import datetime, timezone

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from state import robot, DIRECTION_NAMES, DIRECTION_VECTORS
import sensorsweep as ss
import astar as ast
import frontier as fr
from navigation import start as start_navigation, TURN_90_MS, FORWARD_MS

CALIB_LOG_PATH = os.path.join(os.path.dirname(__file__), "calibration_log.csv")
CALIB_LOG_HEADER = [
    "timestamp", "test_type",
    "commanded_deg", "gyro_deg",
    "commanded_cm", "accel_distance_cm",
    "measured_deg", "measured_cm",
    "motor_left_pct", "motor_right_pct",
    "notes", "test_id",
]

DRIFT_LOG_PATH = os.path.join(os.path.dirname(__file__), "drift_log.csv")
DRIFT_LOG_HEADER = [
    "timestamp", "test_id", "test_category", "commanded_sequence",
    "believed_dx_cm", "believed_dy_cm", "believed_dheading_deg",
    "measured_dx_cm", "measured_dy_cm", "measured_dheading_deg",
    "notes",
]

# Matches robot_firmware.ino's CELL_SIZE_CM (30.48cm / 1ft grid cell).
CELL_SIZE_CM = 30.48

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
    test_type: str  # "left" | "right" | "uturn" | "forward" | "reverse"
    commanded_deg: float | None = None
    gyro_deg: float | None = None
    commanded_cm: float | None = None
    accel_distance_cm: float | None = None
    motor_left_pct: int
    motor_right_pct: int


class CalibMeasured(BaseModel):
    measured_deg: float | None = None
    measured_cm: float | None = None
    notes: str | None = None


class DriftMeasured(BaseModel):
    test_category: str  # "drift_straight" | "drift_turn" | "drift_combined"
    measured_dx_cm: float | None = None
    measured_dy_cm: float | None = None
    measured_dheading_deg: float | None = None
    notes: str | None = None


# ── ESP32 endpoints ───────────────────────────────────────────────────────────

def _apply_sensor_reading(data: SensorData):
    """Shared by the /ws sensor frame handler. Updates robot state + the
    occupancy grid from one HC-SR04 reading (front required, left/right
    optional). Caller must hold robot.lock."""
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


def _handle_obstacle_stop(side: str | None):
    """Shared by the /ws obstacle_stop frame handler. `side` is "left"/"right"
    (angled sensor tripped) or None (front sensor tripped). The queued forward
    command did not complete, so the robot's position did not actually
    advance — don't apply pending_move. Mark the obstacle cell so replanning
    avoids it: the straight-ahead target cell for a front hit, or the
    diagonal ray cell (matching sensorsweep.py's angled-sensor approximation)
    for a side hit. Caller must hold robot.lock."""
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

    if side in ("left", "right"):
        # Distance value here only needs to resolve to "obstacle 1 cell away"
        # in update_from_real_sensor's math — matches robot_firmware.ino's
        # SAFETY_STOP_CM (the threshold that actually tripped this stop).
        ss.update_from_real_sensor(
            robot.x, robot.y, robot.direction,
            15.0, robot.robot_map, side=side
        )
    elif target is not None:
        tx, ty = target
        robot.robot_map[tx, ty] = max(
            -ss.LOG_ODD_CLAMP, min(ss.LOG_ODD_CLAMP, robot.robot_map[tx, ty] + ss.LOG_ODD_HIT)
        )


@app.websocket("/ws")
async def robot_socket(websocket: WebSocket):
    """Single persistent connection replacing the old /sensor_data, /command,
    /stop_flag, and /obstacle_stop HTTP polling — one socket instead of a
    fresh HTTP connect/teardown per action, which is what made the robot feel
    slow to react. The ESP32 sends {"type":"ready"} on connect and after each
    finished command; a writer loop here pops robot.command_queue only while
    esp32_ready is set, pushing the next command the instant it's available.
    A manual stop is pushed immediately too, instead of waiting to be polled."""
    await websocket.accept()

    async def reader():
        while True:
            msg = await websocket.receive_json()
            msg_type = msg.get("type")
            with robot.lock:
                if msg_type == "ready":
                    robot.esp32_ready = True
                elif msg_type == "sensor":
                    _apply_sensor_reading(SensorData(**msg))
                elif msg_type == "obstacle_stop":
                    _handle_obstacle_stop(msg.get("side"))

    async def writer():
        while True:
            with robot.lock:
                if robot.stop_requested:
                    robot.stop_requested = False
                    send = {"type": "stop"}
                elif robot.esp32_ready and robot.command_queue:
                    cmd = robot.command_queue.popleft()
                    cmd["type"] = "command"
                    cmd["left_pct"] = robot.motor_speed_left_pct
                    cmd["right_pct"] = robot.motor_speed_right_pct
                    robot.esp32_ready = False
                    send = cmd
                else:
                    send = None
            if send is not None:
                await websocket.send_json(send)
            await asyncio.sleep(0.02)

    reader_task = asyncio.create_task(reader())
    writer_task = asyncio.create_task(writer())
    try:
        # Either task ending (disconnect surfaces via reader's receive_json)
        # means the connection is done — cancel the other so it doesn't keep
        # running against a dead socket.
        done, pending = await asyncio.wait(
            {reader_task, writer_task}, return_when=asyncio.FIRST_COMPLETED
        )
        for task in pending:
            task.cancel()
        for task in done:
            task.result()
    except WebSocketDisconnect:
        pass
    finally:
        with robot.lock:
            robot.esp32_ready = False


@app.post("/speed")
def set_speed(setting: SpeedSetting):
    """Sets per-side motor PWM duty (%), applied by the ESP32 on its next
    command frame over /ws. Independent left/right (rather than one speed +
    a fixed trim) because measured drift direction wasn't consistent across
    test runs. Clamped to 10-100: too low can't overcome motor static
    friction and stalls without moving."""
    left = max(10, min(100, setting.left_pct))
    right = max(10, min(100, setting.right_pct))
    with robot.lock:
        robot.motor_speed_left_pct = left
        robot.motor_speed_right_pct = right
    return {"status": "ok", "left_pct": left, "right_pct": right}


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
            robot.commanded_sequence.append("left")
            robot.is_moving = True
            return {"status": "ok", "direction": DIRECTION_NAMES[robot.direction]}

        if move.cmd == "right":
            robot.direction = (robot.direction - 1) % 4
            robot.command_queue.append({"cmd": "right", "duration_ms": TURN_90_MS})
            robot.commanded_sequence.append("right")
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
            robot.commanded_sequence.append("forward")
        else:
            robot.command_queue.append({"cmd": "reverse", "duration_ms": FORWARD_MS})
            robot.commanded_sequence.append("reverse")
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
    """ESP32 posts here right after a completed turn (turnByAngle) or a fully-
    completed forward/reverse drive, reporting what the sensors measured.
    Overwrites any previous pending report — only the most recent unlogged
    test can be measured/submitted at a time."""
    with robot.lock:
        robot.pending_calib_report = {
            "test_type": report.test_type,
            "commanded_deg": report.commanded_deg,
            "gyro_deg": report.gyro_deg,
            "commanded_cm": report.commanded_cm,
            "accel_distance_cm": report.accel_distance_cm,
            "motor_left_pct": report.motor_left_pct,
            "motor_right_pct": report.motor_right_pct,
            "test_id": robot.test_id,
        }
    return {"status": "ok"}


@app.get("/calibrate/pending")
def calibrate_pending():
    """Dashboard polls this to know what test is waiting to be measured by
    hand (protractor for turns, tape measure for forward/reverse)."""
    with robot.lock:
        count = robot.calib_log_count
        if robot.pending_calib_report is None:
            return {"pending": False, "logged_count": count}
        return {"pending": True, "logged_count": count, **robot.pending_calib_report}


@app.post("/calibrate/measured")
def calibrate_measured(measured: CalibMeasured):
    """User submits their real measurement(s) for the currently-pending
    report, appending one row to calibration_log.csv for later fitting (see
    calibrate_fit.py). measured_deg/measured_cm are independent — either, both,
    or neither can be filled in by the user depending on what they measured
    for this particular test, regardless of the pending report's test_type."""
    if measured.measured_deg is None and measured.measured_cm is None:
        return {"status": "error", "message": "Enter at least one measurement"}

    with robot.lock:
        if robot.pending_calib_report is None:
            return {"status": "error", "message": "No pending measurement to log"}

        pending = robot.pending_calib_report
        row = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "test_type": pending["test_type"],
            "commanded_deg": pending["commanded_deg"],
            "gyro_deg": pending["gyro_deg"],
            "commanded_cm": pending["commanded_cm"],
            "accel_distance_cm": pending["accel_distance_cm"],
            "measured_deg": measured.measured_deg,
            "measured_cm": measured.measured_cm,
            "motor_left_pct": pending["motor_left_pct"],
            "motor_right_pct": pending["motor_right_pct"],
            "notes": measured.notes,
            "test_id": pending.get("test_id"),
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


@app.post("/calibrate/new_test")
def calibrate_new_test():
    """Starts a new labeled drift-test session: snapshots the robot's currently
    believed pose as the session origin, bumps test_id, and clears the
    commanded-sequence log. Deliberately separate from /map/reset — a map
    reset can happen for unrelated reasons (nav debugging) without meaning
    to start a new test session."""
    with robot.lock:
        robot.test_id += 1
        robot.test_origin = (robot.x, robot.y, robot.direction)
        robot.commanded_sequence = []
        test_id = robot.test_id
    return {"status": "ok", "test_id": test_id}


@app.post("/calibrate/drift/measured")
def calibrate_drift_measured(measured: DriftMeasured):
    """User submits real (tape/protractor) measurements for the drift
    accumulated since the last /calibrate/new_test call. Believed deltas are
    computed from the robot's grid position/direction vs. the session origin;
    measured deltas are whatever the user filled in (independent/optional,
    same pattern as /calibrate/measured — reject only if all three are blank)."""
    if (measured.measured_dx_cm is None and measured.measured_dy_cm is None
            and measured.measured_dheading_deg is None):
        return {"status": "error", "message": "Enter at least one measurement"}

    with robot.lock:
        if robot.test_origin is None:
            return {"status": "error", "message": "No active test session — call /calibrate/new_test first"}

        ox, oy, odir = robot.test_origin
        believed_dx_cm = (robot.x - ox) * CELL_SIZE_CM
        believed_dy_cm = (robot.y - oy) * CELL_SIZE_CM
        # Grid direction only tracks heading mod 4 (90 deg steps) — the
        # shortest signed difference is all that's recoverable, so a session
        # with more than one full net rotation would under-report.
        believed_dheading_deg = (((robot.direction - odir + 2) % 4) - 2) * 90

        row = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "test_id": robot.test_id,
            "test_category": measured.test_category,
            "commanded_sequence": ",".join(robot.commanded_sequence),
            "believed_dx_cm": believed_dx_cm,
            "believed_dy_cm": believed_dy_cm,
            "believed_dheading_deg": believed_dheading_deg,
            "measured_dx_cm": measured.measured_dx_cm,
            "measured_dy_cm": measured.measured_dy_cm,
            "measured_dheading_deg": measured.measured_dheading_deg,
            "notes": measured.notes,
        }
        write_header = not os.path.exists(DRIFT_LOG_PATH)
        with open(DRIFT_LOG_PATH, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=DRIFT_LOG_HEADER)
            if write_header:
                writer.writeheader()
            writer.writerow(row)

    return {"status": "ok"}
