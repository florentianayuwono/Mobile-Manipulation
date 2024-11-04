import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
import pybullet_data

# Connect to PyBullet and load environmentimport time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import Robot
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../")
urdf_dir = os.path.join(root_dir,"resource/urdf")

# mobot_urdf_file = os.path.join(root_dir,"resource/urdf/stretch/stretch.urdf")
# robot_id = Robot(pybullet_api=p, start_pos=[-0.8,0.0,0.05], urdf_file=mobot_urdf_file)
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.1])

mug_position = [0.5, 0, 0.1]
mug_orientation = p.getQuaternionFromEuler([np.pi / 2.0, 0, np.pi - np.pi / 2.0])
mug_scaling = 0.2
mug_id = p.loadURDF(fileName=os.path.join(urdf_dir, "obj_libs/mugs/m2/model.urdf"),
                              useFixedBase=False,
                              globalScaling=mug_scaling,
                              basePosition=mug_position,
                              baseOrientation=mug_orientation)

# object_id = p.loadURDF("random_urdf/000/000.urdf", basePosition=[0.5, 0, 0.1])

p.setGravity(0, 0, -9.8)

# Use inverse kinematics to calculate the joint positions that will move the robot's gripper to the object
target_pos = [0.5, 0, 0.1]
num_joints = p.getNumJoints(robot_id)
end_effector_index = num_joints - 1
joint_positions = p.calculateInverseKinematics(robot_id, end_effector_index, target_pos)
joint_indices = list(range(len(joint_positions)))
p.setJointMotorControlArray(robot_id, joint_indices, p.POSITION_CONTROL, targetPositions=joint_positions)

# Simulate grasping
for _ in range(1000):
    p.stepSimulation()
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# Load robot (using R2D2 as an example)
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.1])

# Ensure the URDF path is correct for the mug object
root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../")
urdf_dir = os.path.join(root_dir, "resource/urdf")

mug_position = [0.5, 0, 0.1]
mug_orientation = p.getQuaternionFromEuler([np.pi / 2.0, 0, np.pi - np.pi / 2.0])
mug_scaling = 0.2

# Check if the mug URDF file exists
mug_urdf_path = os.path.join(urdf_dir, "obj_libs/mugs/m2/model.urdf")
if not os.path.exists(mug_urdf_path):import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import Robot
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../")
urdf_dir = os.path.join(root_dir,"resource/urdf")

# mobot_urdf_file = os.path.join(root_dir,"resource/urdf/stretch/stretch.urdf")
# robot_id = Robot(pybullet_api=p, start_pos=[-0.8,0.0,0.05], urdf_file=mobot_urdf_file)
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.1])

mug_position = [0.5, 0, 0.1]
mug_orientation = p.getQuaternionFromEuler([np.pi / 2.0, 0, np.pi - np.pi / 2.0])
mug_scaling = 0.2
mug_id = p.loadURDF(fileName=os.path.join(urdf_dir, "obj_libs/mugs/m2/model.urdf"),
                              useFixedBase=False,
                              globalScaling=mug_scaling,
                              basePosition=mug_position,
                              baseOrientation=mug_orientation)

# object_id = p.loadURDF("random_urdf/000/000.urdf", basePosition=[0.5, 0, 0.1])

p.setGravity(0, 0, -9.8)

# Use inverse kinematics to calculate the joint positions that will move the robot's gripper to the object
target_pos = [0.5, 0, 0.1]
num_joints = p.getNumJoints(robot_id)
end_effector_index = num_joints - 1
joint_positions = p.calculateInverseKinematics(robot_id, end_effector_index, target_pos)
joint_indices = list(range(len(joint_positions)))
p.setJointMotorControlArray(robot_id, joint_indices, p.POSITION_CONTROL, targetPositions=joint_positions)

# Simulate grasping
for _ in range(1000):
    p.stepSimulation()
    raise FileNotFoundError(f"Mug URDF not found at {mug_urdf_path}")

# Load the mug object
mug_id = p.loadURDF(fileName=mug_urdf_path,
                    useFixedBase=False,
                    globalScaling=mug_scaling,
                    basePosition=mug_position,
                    baseOrientation=mug_orientation)

# Set gravity
p.setGravity(0, 0, -9.8)

# Get the number of joints and identify the end effector (for R2D2 this might differ)
num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: {joint_info}")

# Assuming the last joint is the end effector (this may not apply for R2D2, but for other robots like UR5)
end_effector_index = num_joints - 1

# Define target position for inverse kinematics
target_pos = [0.5, 0, 0.1]

# Use inverse kinematics to calculate joint positions
joint_positions = p.calculateInverseKinematics(robot_id, end_effector_index, target_pos)

# Control the robot's joints
joint_indices = list(range(len(joint_positions)))  # Modify this as needed based on your robot's joints
p.setJointMotorControlArray(robot_id, joint_indices, p.POSITION_CONTROL, targetPositions=joint_positions)

# Simulate the grasping
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)  # Add a small sleep to slow down the simulation
