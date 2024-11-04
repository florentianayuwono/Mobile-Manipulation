import pybullet as p
import time

x_min, x_max = -1, 4  # Adjust according to your environment's dimensions
y_min, y_max = -5, 1
grid_resolution = 0.5



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

def move_to_waypoint(target_pos, robotId, speed=0.25, threshold=0.01):
    """
    Moves the robot towards a given waypoint using wheel motors.

    Parameters:
    - target_pos: List or tuple with target x, y coordinates [x, y]
    - robotId: ID of the robot in the simulation
    - speed: Forward speed multiplier
    - turn_speed: Turning speed multiplier
    - threshold: Distance to the target below which the robot will stop
    """
    turn_speed = 0.5 * speed  # Reduce turning speed for smoother motion
    # Loop until the robot reaches the target position within the threshold
    while True:
        # Get current position and orientation
        position, orientation = p.getBasePositionAndOrientation(robotId)
        current_pos = np.array(position[:2])  # Only x, y for 2D navigation

        # Calculate the direction and distance to the target
        target_vector = np.array(target_pos[:2]) - current_pos
        distance = np.linalg.norm(target_vector)
        
        if distance <= threshold:
            # Stop the robot if it is close enough to the waypoint
            p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=1000)
            p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=1000)
            break

        # Get the robot's current heading
        _, _, yaw = p.getEulerFromQuaternion(orientation)
        target_angle = np.arctan2(target_vector[1], target_vector[0])

        # Calculate the angle difference and normalize to [-pi, pi]
        angle_diff = target_angle - yaw
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

        # Determine wheel velocities based on angle difference
        if abs(angle_diff) > 0.1:  # Turn towards the target if not facing it
            leftWheelVelocity = -turn_speed * np.sign(angle_diff)
            rightWheelVelocity = turn_speed * np.sign(angle_diff)
        else:  # Move forward if facing the target
            leftWheelVelocity = speed
            rightWheelVelocity = speed

        # Set motor velocities
        p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=leftWheelVelocity, force=1000)
        p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=rightWheelVelocity, force=1000)

        # Step simulation
        p.stepSimulation()
        time.sleep(1./240.)  # Adjust as necessary for smoother movement

def find_path(start_pos, goal_pos, obstacles):

    def is_occupied(x, y):
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
def move_to_waypoint(target_pos, robotId):
    position, orientation = p.getBasePositionAndOrientation(robotId)
    p.resetBasePositionAndOrientation(robotId, [target_pos[0], target_pos[1],0.1], orientation)



# example usage
# start_pos = (-1, 0)
# goal_pos = (3, 0)
# path = find_path(start_pos, goal_pos)
# if path:
#     for pos in path:
#         move_to_waypoint(pos, mobot.robotId)
#         p.stepSimulation()
#         time.sleep(1./240.)