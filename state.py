import threading
import numpy as np
from collections import deque


# Direction constants: index into DIRECTION_VECTORS
# 0=East(+x), 1=North(+y), 2=West(-x), 3=South(-y)
DIRECTION_VECTORS = [(1, 0), (0, 1), (-1, 0), (0, -1)]
DIRECTION_NAMES   = ["East", "North", "West", "South"]


class RobotState:
    def __init__(self):
        self.grid_size = (50, 50)
        self.x = 0
        self.y = 0
        self.direction = 0          # starts facing East
        self.path = []
        self.is_moving = False
        self.goal = None
        self.robot_map = np.full(self.grid_size, 0.0)
        self.world_map = np.zeros(self.grid_size)   # kept for fallback/sim

        # "idle" | "manual" | "goto" | "explore" — mutually exclusive drive modes
        self.mode = "idle"
        self.lock = threading.Lock()

        # Real sensor data from ESP32
        self.sensor_distance_cm = 400.0  # last front-sensor reading
        self.sensor_updated = False       # True once first real reading arrives

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


robot = RobotState()
