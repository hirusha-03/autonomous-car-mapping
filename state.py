import threading
import numpy as np


class RobotState:
   def __init__(self):
        self.grid_size = (10, 10)
        self.x = 0
        self.y = 0
        self.path = []
        self.is_moving = False
        self.goal = None
        self.robot_map = np.full(self.grid_size, 0.0)
        self.world_map = np.zeros(self.grid_size)
        self.world_map[2:5, 5:7] = 1
        self.exploring = False
        self.lock = threading.Lock()

robot = RobotState()    