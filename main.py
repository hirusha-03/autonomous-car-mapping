from fastapi import FastAPI
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from state import robot
import sensorsweep as ss
import astar as ast
import frontier as fr
from navigation import start as start_navigation

app = FastAPI()

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

@app.on_event("startup")
def startup():
    ss.sensor_sweep(robot.x, robot.y, robot.world_map, robot.robot_map, sensor_range=2)
    start_navigation()

@app.get("/map")  # update existing /map to include exploring state
def get_map():
    with robot.lock:
        return {
            "robot_position": (robot.x, robot.y),
            "robot_map": robot.robot_map.tolist(),
            "world_map": robot.world_map.tolist(),
            "path": robot.path,
            "is_moving": robot.is_moving,
            "exploring": robot.exploring,         # add this
            "frontiers": fr.find_frontiers(        # add this — renders in UI
                robot.robot_map, robot.grid_size
            )
        }

@app.post("/navigate")
def navigate(target: Target):
    with robot.lock:

        # Check boundary
        if not (0 <= target.x < robot.grid_size[0] and 0 <= target.y < robot.grid_size[1]):
            return {"status": "error", "message": "Target out of bounds"}

        # Check if target is known obstacle
        if robot.robot_map[target.x, target.y] >= 1.386:
            return {"status": "error", "message": "Target is an obstacle"}

        # Check if already moving
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

        robot.path = result[1:]
        return {"status": "ok", "path": robot.path}

@app.get("/status")
def get_status():
    with robot.lock:
        return {
            "is_moving": robot.is_moving,
            "x": robot.x,
            "y": robot.y,
            "goal": robot.goal
        }

@app.post("/explore/start")
def start_explore():
    with robot.lock:
        if robot.is_moving:
            return {"status": "error", "message": "Robot is moving"}
        robot.exploring = True
        robot.goal = None
        robot.path = []
        return {"status": "ok", "message": "Autonomous exploration started"}

@app.post("/explore/stop")
def stop_explore():
    with robot.lock:
        robot.exploring = False
        robot.path = []
        robot.goal = None
        robot.is_moving = False
        return {"status": "ok", "message": "Exploration stopped"}

