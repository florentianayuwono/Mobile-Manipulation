import os
import pybullet as p
import pybullet_data
import time
from stretch import Robot

# Initialize PyBullet in GUI mode
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Set up the environment
plane_id = p.loadURDF("plane.urdf")
root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../")
mobot_urdf_file = os.path.join(root_dir, "resource/urdf/stretch/stretch.urdf")

# Initialize the robot with a starting position
mobot = Robot(pybullet_api=p, start_pos=[1, 0, 0.2], urdf_file=mobot_urdf_file)
cube_id = p.loadURDF("cube.urdf", basePosition=[2, 0, 0.2], globalScaling=0.1)

# Helper function to move the robot base closer to the object
def move_robot_base(robot, target_position, steps=100):
    # Calculate stepwise increments for a smooth approach
    start_pos, _ = p.getBasePositionAndOrientation(robot.robotId)
    step_size = [(t - s) / steps for s, t in zip(start_pos, target_position)]

    for _ in range(steps):
        # Incrementally move the robot base
        new_position = [s + step_size[i] for i, s in enumerate(start_pos)]
        p.resetBasePositionAndOrientation(robot.robotId, new_position, [0, 0, 0, 1])
        p.stepSimulation()
        time.sleep(1. / 240.)
        start_pos = new_position

# Function to approach and grasp the cube
def grasp_object(robot, object_id, robot_link_index=10, approach_position=[2, 0, 0.2], grasp_position=[2, 0, 0.2]):
    # Step 1: Move the end effector close to the object
    approach_angles = p.calculateInverseKinematics(robot.robotId, robot_link_index, approach_position)
    for i, angle in enumerate(approach_angles):
        p.setJointMotorControl2(robot.robotId, i, p.POSITION_CONTROL, targetPosition=angle)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1. / 240.)

    # Step 2: Close the gripper fingers
    left_finger_index = 14
    right_finger_index = 15
    p.setJointMotorControl2(robot.robotId, left_finger_index, p.POSITION_CONTROL, targetPosition=-0.02)
    p.setJointMotorControl2(robot.robotId, right_finger_index, p.POSITION_CONTROL, targetPosition=0.02)

    for _ in range(100):  # Allow some time for the gripper to close
        p.stepSimulation()
        time.sleep(1. / 240.)

    # Step 3: Apply a fixed constraint to lock the grasp
    constraint_id = p.createConstraint(
        parentBodyUniqueId=robot.robotId,
        parentLinkIndex=robot_link_index,
        childBodyUniqueId=object_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )
    return constraint_id

# Move the robot base closer to the cube before attempting the grasp
move_robot_base(mobot, target_position=[1.8, 0, 0.2])

# Perform grasping
grasp_constraint = grasp_object(mobot, cube_id)

# Lift the cube by moving the arm upwards
lift_position = [2, 0, 0.5]
lift_angles = p.calculateInverseKinematics(mobot.robotId, 10, lift_position)
for i, angle in enumerate(lift_angles):
    p.setJointMotorControl2(mobot.robotId, i, p.POSITION_CONTROL, targetPosition=angle)

# Run the simulation
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1. / 240.)

# Clean up
p.removeConstraint(grasp_constraint)
p.disconnect()
