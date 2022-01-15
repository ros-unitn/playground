#! /usr/bin/env python3

import itertools
import random
from itertools import product
from pathlib import Path

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from geometry_msgs.msg import Pose, Point, Quaternion

block_2x2 = "X2-Y2-Z2"

file_path = Path(__file__)
playground_path = file_path.parents[3]
models_path = playground_path.joinpath("models", "blocks.model")

x_space_layer_1 = [0.485, 0.55, 0.615]
y_space_layer_1 = [0.065, 0.0, -0.065]

x_space_layer_2 = [0.5175, 0.5825]
y_space_layer_2 = [0.0325, -0.0325]

table_height = 0.72

buffer_height = 0.01
block_2_height = 0.05 + buffer_height

def spawn_model(name, pos=Point(0.6, 0, 0.72), quat=Quaternion(0, 0, 0, 0)):
    req = SpawnModelRequest()
    req.model_name = "%s_%s" % (name, random.randint(1, 1000))
    with open(models_path.joinpath(name, "model.sdf")) as f:
        req.model_xml = f.read()
    req.initial_pose = Pose(pos, quat)
    res: SpawnModelResponse = service.call(req)
    if not res.success:
        raise RuntimeError(res.status_message)


if __name__ == "__main__":
    rospy.init_node("x_world_node")
    service = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    for (x, y) in product(x_space_layer_1, y_space_layer_1):
        spawn_model(block_2x2, pos=Point(x, y, table_height))

    for (x, y) in product(x_space_layer_2, y_space_layer_2):
        spawn_model(block_2x2, pos=Point(x, y, table_height + block_2_height))
