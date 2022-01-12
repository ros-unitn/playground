#! /usr/bin/env python3

from pathlib import Path

import rospy
from gazebo_msgs.srv import (
    GetWorldProperties,
    GetWorldPropertiesRequest,
    GetWorldPropertiesResponse,
    DeleteModel,
    DeleteModelRequest,
    DeleteModelResponse,
)

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


if __name__ == "__main__":
    rospy.init_node("x_world_node")
    world_service = rospy.ServiceProxy(
        "gazebo/get_world_properties", GetWorldProperties
    )
    delete_service = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    res: GetWorldPropertiesResponse = world_service.call(GetWorldPropertiesRequest())
    for name in res.model_names:
        if any([default in name for default in names]):
            res: DeleteModelResponse = delete_service.call(DeleteModelRequest(name))
            if not res.success:
                raise RuntimeError(res.status_message)
