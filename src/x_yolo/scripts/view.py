#! /usr/bin/env python3

import sys
from pathlib import Path

import rospy
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

file_path = Path(__file__)
package_path = file_path.parents[1]

yolo_repo_path = Path(package_path, "yolov5")
yolo_weights_path = Path(package_path, "best.pt")

bridge = CvBridge()

# add yolo to PYTHONPATH
if not yolo_repo_path in sys.path:
    sys.path.append(str(yolo_repo_path))

if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = torch.hub.load(
        yolo_repo_path, "custom", path=yolo_weights_path, source="local"
    )  # local repo

    rospy.init_node("x_yolo")

    raw_color = rospy.wait_for_message("/camera/color/image_raw", Image)
    color = bridge.imgmsg_to_cv2(raw_color, "rgb8")
    res = model(color)

    res.show()
