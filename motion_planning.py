import pybullet as p
import time

x_min, x_max = -1, 4  # Adjust according to your environment's dimensions
y_min, y_max = -5, 1
grid_resolution = 0.5

def is_occupied(x, y, obstacles):
    # Create a temporary collision shape representing the robot's footprint
    collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=grid_resolution/2, height=1.0)
    test_body = p.createMultiBody(baseCollisionShapeIndex=collision_shape, basePosition=[x, y, 0.05])

    # Check for collisions with obstacles
    collision = False
    for obstacle_id in obstacles:
        contact_points = p.getClosestPoints(test_body, obstacle_id, distance=0)
        if contact_points:
            collision = True
            break
    
    # Clean up the temporary body
    # if collision:
    p.removeBody(test_body)
    return collision


def pos_to_idx(x, y, grid_resolution=0.5):
    i = int(round((x - x_min) / grid_resolution))
    j = int(round((y - y_min) / grid_resolution))
    return (i, j)

def idx_to_pos(i, j):
    x = x_min + i * grid_resolution
    y = y_min + j * grid_resolution
    return (x, y)

# Calculate grid dimensions
nx = int(round((x_max - x_min) / grid_resolution)) + 1
ny = int(round((y_max - y_min) / grid_resolution)) + 1





import numpy as np
import heapq

def heuristic(a, b):
    # Manhattan distance heuristic for grid-based movement
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar_grid(grid, start_idx, goal_idx):
    """
    Performs A* pathfinding on a 2D grid.

    Parameters:
    - grid: 2D numpy array where 1 represents free space and 0 represents obstacles.
    - start_idx: Tuple (i, j) indicating the starting cell indices.
    - goal_idx: Tuple (i, j) indicating the goal cell indices.

    Returns:
    - path: List of (i, j) tuples representing the path from start to goal.
    """
    open_set = []
    heapq.heappush(open_set, (0, start_idx))
    came_from = {}

    g_score = {start_idx: 0}
    f_score = {start_idx: heuristic(start_idx, goal_idx)}

    while open_set:
        _, current = heapq.heappop(open_set)

   
        if current[0] == goal_idx[0] and current[1] == goal_idx[1]:
            # Reconstruct the path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_idx)
            path.reverse()
            return path

        # Define neighbor positions (4-connected grid)
        neighbors = [
            (current[0] + dx, current[1] + dy)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        ]

        for neighbor in neighbors:
            i, j = neighbor
            if 0 <= i < nx and 0 <= j < ny:
                if grid[i, j] == 0:
                    continue  # Skip obstacles
                tentative_g_score = g_score[current] + 1  # Assume uniform cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_idx)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

def idx_to_pos(i, j):
    x = x_min + i * grid_resolution
    y = y_min + j * grid_resolution
    return (x, y)

def pos_to_idx(x, y):
    i = int(round((x - x_min) / grid_resolution))
    j = int(round((y - y_min) / grid_resolution))
    return (i, j)

# Example usage:

# Get robot's position
start_pos = (-1, 0)

# Get cup's position
goal_pos = (3, 0)

def find_path(start_pos, goal_pos):
    # create a 2d grid
    grid = np.ones((nx, ny))
    for i in range(nx):
        for j in range(ny):
            x, y = idx_to_pos(i, j)
            if is_occupied(x, y):
                grid[i, j] = 0

    # Convert positions to grid indices
    start_idx = pos_to_idx(*start_pos)
    goal_idx = pos_to_idx(*goal_pos)

    # Ensure start and goal indices are within grid bounds
    if not (0 <= start_idx[0] < nx and 0 <= start_idx[1] < ny):
        print("Start position is out of grid bounds.")
    if not (0 <= goal_idx[0] < nx and 0 <= goal_idx[1] < ny):
        print("Goal position is out of grid bounds.")

    # Ensure start and goal positions are not inside obstacles
    if grid[start_idx[0], start_idx[1]] == 0:
        print("Start position is inside an obstacle.")
    if grid[goal_idx[0], goal_idx[1]] == 0:
        print("Goal position is inside an obstacle.")

    # Run the A* algorithm
    path_indices = astar_grid(grid, start_idx, goal_idx)
    if path_indices is None:
        print("No path found.")
        return None
    else:
        print(f"Path found with {len(path_indices)} steps.")

        # Convert path indices back to world positions
        path = [idx_to_pos(i, j) for (i, j) in path_indices]
    
        # Draw the path in PyBullet
        for i in range(len(path) - 1):
            start_point = [path[i][0], path[i][1], 0.1]  # Start point (with slight Z offset to make it visible)
            end_point = [path[i + 1][0], path[i + 1][1], 0.1]  # End point (with slight Z offset)
            
            # Draw a line between consecutive waypoints
            p.addUserDebugLine(
                start_point, 
                end_point, 
                lineColorRGB=[1, 0, 0],  # Red color for the path
                lineWidth=3  # Optional: Line width
            )

        return path


# write a move to waypoint function
def move_to_waypoint(target_pos, robotId, steps):
    position, orientation = p.getBasePositionAndOrientation(robotId)
    # p.resetBasePositionAndOrientation(robotId, [target_pos[0], target_pos[1],0.1], orientation)
    start_pos, _ = p.getBasePositionAndOrientation(robotId)
    step_size = [(t - s) / steps for s, t in zip(start_pos, target_pos)]

    for _ in range(steps):
        # Incrementally move the robot base
        new_position = [s + step_size[i] for i, s in enumerate(start_pos)]
        p.resetBasePositionAndOrientation(robotId, new_position, orientation)
        p.stepSimulation()
        time.sleep(1. / 240.)
        start_pos = new_position


# example usage
# start_pos = (-1, 0)
# goal_pos = (3, 0)
# path = find_path(start_pos, goal_pos)
# if path:
#     for pos in path:
#         move_to_waypoint(pos, mobot.robotId)
#         p.stepSimulation()
#         time.sleep(1./240.)
