#! /usr/bin/env python3

import random
from pathlib import Path

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from geometry_msgs.msg import Pose, Point, Quaternion

names = [
    "X1-Y1-Z2",
    "X1-Y2-Z1",
    "X1-Y2-Z2",
    "X1-Y2-Z2-CHAMFER",
    "X1-Y2-Z2-TWINFILLET",
    "X1-Y3-Z2",
    "X1-Y3-Z2-FILLET",
    "X1-Y4-Z1",
    "X1-Y4-Z2",
    "X2-Y2-Z2",
    "X2-Y2-Z2-FILLET",
]

file_path = Path(__file__)
playground_path = file_path.parents[3]
models_path = playground_path.joinpath("models", "blocks.model")


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
    name = random.choice(names)
    spawn_model(name)
