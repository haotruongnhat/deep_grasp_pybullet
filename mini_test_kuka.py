import numpy as np
import pybullet as p
import pybullet_data
from collections import namedtuple

def generateFloor(size=1):
    # Load URDF file and colour
    visualShapeId = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName='floor.obj',
        rgbaColor=[0.6, 0.6, 0.6, 1],  # grey
        meshScale=[size, size, size])

    # Ensure floor can be collided with
    collisionShapeId = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName='floor.obj',
        meshScale=[size, size, size])

    # Merge collision and visual profile and place object
    floor = p.createMultiBody(
        baseCollisionShapeIndex=collisionShapeId,
        baseVisualShapeIndex=visualShapeId,
        basePosition=[0, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([np.deg2rad(90), 0, 0]))

    return floor

def generateRobot(jointPositions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
    # Load KUKA iiwa Model
    start_pos = [0, 0, 0]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    kuka = p.loadURDF("kuka_iiwa/model.urdf", start_pos, start_orientation)

    # Set KUKA Joint angles
    for jointIndex in np.arange(p.getNumJoints(kuka)):
        p.resetJointState(kuka, jointIndex, jointPositions[jointIndex])
        p.setJointMotorControl2(bodyUniqueId=kuka, jointIndex=jointIndex, controlMode=p.POSITION_CONTROL, maxVelocity=KUKA_VELOCITY_LIMIT)

    lower_limits = []
    upper_limits = []
    for joint in range(p.getNumJoints(kuka)):
        info = p.getJointInfo(kuka, joint)
        lower_limits.append(info[8])
        upper_limits.append(info[9])

    # Check number of joint inputs is correct
    if len(jointPositions) != p.getNumJoints(kuka):
        pass
    else:
        # Check inputs are within maximum and minimum joint limits and amend if necessary
        for joint in range(p.getNumJoints(kuka)):
            if jointPositions[joint] < lower_limits[joint]:
                jointPositions[joint] = lower_limits[joint]

            elif jointPositions[joint] > upper_limits[joint]:
                jointPositions[joint] = upper_limits[joint]

        p.setJointMotorControlArray(kuka, range(p.getNumJoints(kuka)), p.POSITION_CONTROL,
                                    jointPositions)


    return kuka

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# todo verify ranges in simulation and minimum height
KUKA_MIN_RANGE = .400               # approximation based on robot documentation
KUKA_MAX_RANGE = .780               # approximation based on robot documentation
KUKA_VELOCITY_LIMIT = 1.0           # Maximum velocity of robotic joints. If not lmiited then robot goes out of its joint limit
POSITIONAL_TOLERANCE = 0.001  # 1mm
ROTATIONAL_TOLERANCE = 1.0  # 1 degree
VERTICAL_DISTANCE = 0.05  # 5cm

SIMULATION_STEP_DELTA = 1. / 240.   # Time between each simulation step
SIMULATION_STEPS_PER_TIMESTEP = 24  # Number of simulation steps in one algorithmic timestep

p.setTimeStep(SIMULATION_STEP_DELTA)
p.loadURDF("plane.urdf", [0, 0, -1])
p.setGravity(0, 0, -10)

# Load objects
positions = [1.5, 1.5, 1.5, 1.5, 1.5, -1.5, 0]
kuka_id  = generateRobot(positions)
floor_id = generateFloor()

# Returns an RGB image from the Eye In Hand camera on the robot's end effector
# Credit: https://github.com/bulletphysics/bullet3/issues/1616
def getEyeInHandCamera():
    eih_res = 112
    eih_dep = 3

    # Set up camera positioning
    fov, aspect, nearplane, farplane = 50, 1.0, 0.01, 100
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

    # Center of mass position and orientation (of link-7)
    com_p, com_o, _, _, _, _ = p.getLinkState(kuka_id, 6, computeForwardKinematics=True)
    rot_matrix = p.getMatrixFromQuaternion(com_o)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    # Initial vectors
    init_camera_vector = (0, 0, 1)  # z-axis
    init_up_vector = (0, 1, 0)  # y-axis
    # Rotated vectors
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)
    view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
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

boxID = p.loadURDF("./urdf/skew-box-button.urdf",
                        [0.5, 0.4, 0.0],
                        # p.getQuaternionFromEuler([0, 1.5706453, 0]),
                        p.getQuaternionFromEuler([0, 0, 0]),
                        useFixedBase=True,
                        flags=p.URDF_MERGE_FIXED_LINKS | p.URDF_USE_SELF_COLLISION)

while True:
    p.stepSimulation()
    rgb = getEyeInHandCamera()

    obs = dict()
    obs.update(dict(rgb=rgb))