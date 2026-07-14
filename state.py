import threading
import numpy as np
from collections import deque


# Direction constants: index into DIRECTION_VECTORS
# 0=East(+x), 1=North(+y), 2=West(-x), 3=South(-y)
DIRECTION_VECTORS = [(1, 0), (0, 1), (-1, 0), (0, -1)]
DIRECTION_NAMES   = ["East", "North", "West", "South"]


class RobotState:
    def __init__(self):
        self.grid_size = (25, 25)
        self.x = self.grid_size[0] // 2
        self.y = self.grid_size[1] // 2
        self.direction = 0          # starts facing East
        self.path = []
        self.is_moving = False
        self.goal = None
        self.robot_map = np.full(self.grid_size, 0.0)
        self.world_map = np.zeros(self.grid_size)   # kept for fallback/sim

        # "idle" | "manual" | "goto" | "explore" — mutually exclusive drive modes
        self.mode = "idle"
        self.lock = threading.Lock()

        # Real sensor data from ESP32 — front is required, left/right are
        # optional angled (~30deg) side sensors, default to "clear" until seen.
        self.sensor_distance_cm = 400.0       # last front-sensor reading
        self.sensor_distance_left_cm = 400.0
        self.sensor_distance_right_cm = 400.0
        self.sensor_updated = False       # True once first real reading arrives

        # MPU6050 IMU — accel in m/s^2, gyro_z (yaw rate) in rad/s. Defaults to
        # 0 until the first real reading arrives (mirrors sensor_updated).
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_z = 0.0

        # Gyro turn-calibration workflow: ESP32 reports what the gyro measured
        # for a turn, user measures the real angle with a protractor and logs
        # it via the dashboard — see /calibrate/report and /calibrate/measured
        # in main.py. pending_calib_report holds the not-yet-measured report
        # (None once logged or if nothing has been reported yet).
        self.pending_calib_report = None
        self.calib_log_count = 0

        # Multi-cell drift test session (Part 4): /calibrate/new_test snapshots
        # test_origin (believed pose at session start) and bumps test_id so
        # per-command calibration rows can be joined against drift_log.csv
        # rows later. Decoupled from /map/reset on purpose — a map reset can
        # happen for unrelated reasons (nav debugging) without starting a new
        # labeled test session.
        self.test_id = 0
        self.test_origin = None  # (x, y, direction) or None before first new_test
        self.commanded_sequence = []  # commands issued since the last new_test

        # Motor command queue — navigation fills it, ESP32 drains it
        # Each entry: {"cmd": str, "duration_ms": int}
        self.command_queue: deque = deque()

        # (x, y) the last enqueued "forward" command will land on, applied once
        # the queue drains. Avoids relying on stale loop-local variables.
        self.pending_move = None

        # Edge-triggered abort flag: set by /manual/move stop, consumed (and
        # cleared) by the ESP32's mid-motion /stop_flag poll so a manual stop
        # can interrupt a drive/turn already in progress instead of waiting
        # for the current command's full duration_ms to elapse.
        self.stop_requested = False

        # Independent motor PWM duty per side (10-100), applied by the ESP32
        # to ENA (left)/ENB (right). Separate rather than one speed + trim
        # because measured drift direction wasn't consistent across test
        # runs (see ai_context/INDEX.md Hardware Calibration Log) — a fixed
        # one-directional trim would assume a bias that doesn't always hold.
        # Attached fresh to every /command response rather than threaded
        # through each enqueue call site, so a change takes effect on the
        # very next command the robot executes.
        self.motor_speed_left_pct = 70
        self.motor_speed_right_pct = 70

    def reset(self):
        """Wipe the map and drive state back to a fresh start, keeping the
        current sensor/PWM calibration (those reflect the physical robot,
        not the map, so a map reset shouldn't discard them)."""
        self.x = self.grid_size[0] // 2
        self.y = self.grid_size[1] // 2
        self.direction = 0
        self.path = []
        self.is_moving = False
        self.goal = None
        self.robot_map = np.full(self.grid_size, 0.0)
        self.world_map = np.zeros(self.grid_size)
        self.mode = "idle"
        self.command_queue.clear()
        self.pending_move = None
        self.stop_requested = False


robot = RobotState()
