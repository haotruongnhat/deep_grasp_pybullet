import os

import numpy as np
import pybullet as p
import pybullet_data

from robot import UR5Robotiq85
import time
import math

SIMULATION_STEP_DELAY = 1 / 240.

def step_simulation():
    """
    Hook p.stepSimulation()
    """
    p.stepSimulation()
    time.sleep(SIMULATION_STEP_DELAY)

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

planeID = p.loadURDF("plane.urdf")

action = [0.7, 0, 0.8, 0, np.pi/2, -np.pi/2, 0.01]

robot = UR5Robotiq85((0, 0, 0), (0, 0, 0))
robot.load()

robot.step_simulation = step_simulation

# Returns an RGB image from the Eye In Hand camera on the robot's end effector
# Credit: https://github.com/bulletphysics/bullet3/issues/1616
def getEyeInHandCamera():
    eih_res = 320
    eih_dep = 3

    # Set up camera positioning
    fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

    # Center of mass position and orientation (of link-7)
    com_p, com_o, _, _, _, _ = p.getLinkState(robot.id, 8, computeForwardKinematics=True)
    rot_matrix = p.getMatrixFromQuaternion(com_o)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    # Initial vectors
    init_camera_vector = (0, 0, 1)  # z-axis
    init_up_vector = (0, 1, 0)  # y-axis
    # Rotated vectors
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)
    view_matrix = p.computeViewMatrix(com_p + 0.13 * camera_vector, com_p + 0.3 * camera_vector, up_vector)
    img = p.getCameraImage(eih_res, eih_res, view_matrix,
                            projection_matrix)

    return pixelArray2RGBArray(img[2], eih_dep, eih_res)

def pixelArray2RGBArray(rgba_seq, img_dep, img_res):
    r, g, b = [], [], []
    for row in rgba_seq:
        for pixel in row:
            r.append(pixel[0])
            g.append(pixel[1])
            b.append(pixel[2])

    rgb = [r, g, b]
    # Split into three RGB layers
    return np.array(rgb).reshape(img_dep, img_res, img_res)

step = 0

while True:
    robot.move_ee(action[:-1], "end")
    robot.move_gripper(action[-1])

    rgb = getEyeInHandCamera()
    
    for _ in range(120):  # Wait for a few steps
        step_simulation()
    
    if step == 2:
        boxID = p.loadURDF("./urdf/skew-box-button.urdf",
                        [0.7, 0.0, 0.0],
                        # p.getQuaternionFromEuler([0, 1.5706453, 0]),
                        p.getQuaternionFromEuler([0, 0, 0]),
                        useFixedBase=True,
                        flags=p.URDF_MERGE_FIXED_LINKS | p.URDF_USE_SELF_COLLISION)

    step += 1
