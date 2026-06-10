import numpy as np
from collections import deque

UNKNOWN_THRESHOLD = 0.5   # |log-odds| below this = unexplored
FREE_THRESHOLD    = -0.5  # log-odds below this = known free

def find_frontiers(robot_map, grid_size):
    """
    Returns list of (x, y) frontier cells — unknown cells that
    border at least one known-free cell. Sorted by cluster size
    (largest frontier region first).
    """
    rows, cols = grid_size
    visited = set()
    frontiers = []

    for x in range(rows):
        for y in range(cols):
            if (x, y) in visited:
                continue
            if not _is_frontier(x, y, robot_map):
                continue

            # BFS to collect the full connected frontier cluster
            cluster = []
            queue = deque([(x, y)])
            visited.add((x, y))

            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
                    nx, ny = cx+dx, cy+dy
                    if (nx, ny) not in visited \
                       and 0 <= nx < rows \
                       and 0 <= ny < cols \
                       and _is_frontier(nx, ny, robot_map):
                        visited.add((nx, ny))
                        queue.append((nx, ny))

            frontiers.append(cluster)

    # Sort: largest cluster first (more unexplored area = higher priority)
    frontiers.sort(key=len, reverse=True)
    return frontiers


def best_frontier_target(robot_map, robot_pos, grid_size):
    """
    Returns the single best (x, y) cell to navigate toward.
    Strategy: centroid of the largest frontier cluster.
    Falls back to nearest frontier cell if centroid is an obstacle.
    """
    clusters = find_frontiers(robot_map, grid_size)
    if not clusters:
        return None  # fully explored

    rx, ry = robot_pos

    for cluster in clusters:
        # Try the centroid of this cluster
        cx = int(np.mean([c[0] for c in cluster]))
        cy = int(np.mean([c[1] for c in cluster]))

        if robot_map[cx, cy] < 1.386:  # not an obstacle
            return (cx, cy)

        # Centroid blocked — pick cluster cell closest to robot
        best = min(cluster, key=lambda c: abs(c[0]-rx) + abs(c[1]-ry))
        return best

    return None


def _is_frontier(x, y, robot_map):
    """A cell is a frontier if it's unknown AND borders a known-free cell."""
    if abs(robot_map[x, y]) > UNKNOWN_THRESHOLD:
        return False  # already known (free or obstacle)

    rows, cols = robot_map.shape
    for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
        nx, ny = x+dx, y+dy
        if 0 <= nx < rows and 0 <= ny < cols:
            if robot_map[nx, ny] < FREE_THRESHOLD:
                return True  # has a known-free neighbor
    return False