#! /usr/bin/env python3

import itertools
import random
import math
from itertools import product
from pathlib import Path

from numpy import block

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from geometry_msgs.msg import Pose, Point, Quaternion

block_2x2 = "X2-Y2-Z2"

file_path = Path(__file__)
playground_path = file_path.parents[3]
models_path = playground_path.joinpath("models", "blocks.model")

x_space = [0.50, 0.60, 0.70]
y_space = [0.20, 0.10, 0.0, -0.10, -0.20]
z = 0.8

def spawn_model(name, pos=Point(0.6, 0, 0.72), quat=Quaternion(0, 0, 0, 0)):
    req = SpawnModelRequest()
    req.model_name = "%s_%s" % (name, random.randint(1, 10000))
    with open(models_path.joinpath(name, "model.sdf")) as f:
        req.model_xml = f.read()
    req.initial_pose = Pose(pos, quat)
    res: SpawnModelResponse = service.call(req)
    if not res.success:
        raise RuntimeError(res.status_message)


if __name__ == "__main__":
    rospy.init_node("x_world_node")
    service = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    random_positions = list(product(x_space, y_space))
    random.shuffle(random_positions)
    random_positions = random_positions[:15]

    for (x, y) in random_positions:
        # qx, qy, qz, qw = [random.random() * 2 * math.pi for _ in range(4)]
        qx, qy, qz, qw = 0, 0, 0, 0
        spawn_model(block_2x2, pos=Point(x, y, z), quat=Quaternion(qx, qy, qz, qw))
