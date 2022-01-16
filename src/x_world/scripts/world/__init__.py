#! /usr/bin/env python3#! /usr/bin/env python3

import random
from pathlib import Path
import xml.etree.ElementTree as ET

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from geometry_msgs.msg import Pose, Point, Quaternion

file_path = Path(__file__)
playground_path = file_path.parents[5]
models_path = playground_path.joinpath("models", "blocks.model")

materials = [
    "Gazebo/Grey",
    "Gazebo/White",
    "Gazebo/Red",
    "Gazebo/RedBright",
    "Gazebo/Green",
    "Gazebo/Blue",
    "Gazebo/SkyBlue",
    "Gazebo/Yellow",
    "Gazebo/ZincYellow",
    "Gazebo/DarkYellow",
    "Gazebo/Purple",
    "Gazebo/Turquoise",
    "Gazebo/Orange",
    "Gazebo/Indigo",
]


material_xml = "<material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{}</name></script></material>"


class Spawner:
    def __init__(self):
        rospy.init_node("x_world_node")
        self.service = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    def spawn_model(self, name, pos=Point(0.6, 0, 0.72), quat=Quaternion(0, 0, 0, 0)):
        req = SpawnModelRequest()
        req.model_name = "%s_%s" % (name, random.randint(1, 1000))

        with open(models_path.joinpath(name, "model.sdf")) as f:
            sdf = ET.fromstring(f.read())
            model = sdf.find("model")
            link = model.find("link")
            visual = link.find("visual")
            material = material_xml.format(random.choice(materials))
            visual.append(ET.fromstring(material))
            req.model_xml = ET.tostring(sdf).decode()

        req.initial_pose = Pose(pos, quat)
        res: SpawnModelResponse = self.service.call(req)
        if not res.success:
            raise RuntimeError(res.status_message)
