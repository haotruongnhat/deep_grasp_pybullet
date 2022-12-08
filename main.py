import os

import numpy as np
import pybullet as p

from tqdm import tqdm
from env import ClutteredPushGrasp
from robot import Panda, UR5Robotiq85, UR5Robotiq140
from utilities import YCBModels, Camera, AttachedCamera
import time
import math


def user_control_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '**', 'textured-decmp.obj'),
    )
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    camera = AttachedCamera(robot, 0.1, 5, 1.0, 60)
    env = ClutteredPushGrasp(robot, ycb_models, camera, vis=True)

    env.reset()
    while True:
        obs, reward, done, info = env.step(env.read_debug_parameter(), 'end')


if __name__ == '__main__':
    user_control_demo()
