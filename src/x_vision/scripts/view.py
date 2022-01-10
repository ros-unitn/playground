#! /usr/bin/env python3

import sys
from threading import Lock
from pathlib import Path

import cv2
import rospy
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

file_path = Path(__file__)
package_path = file_path.parents[1]

yolo_repo_path = Path(package_path, "yolov5")
yolo_weights_path = Path(package_path, "best.pt")

bridge = CvBridge()

image_lock = Lock()
image = None

# add yolo to PYTHONPATH
if not yolo_repo_path in sys.path:
    sys.path.append(str(yolo_repo_path))

def raw_color_callback(img: Image):
    global image
    color = bridge.imgmsg_to_cv2(img, "bgr8")
    res = model(color)
    res.render()
    if image_lock.acquire(blocking=False):
        image = res.imgs[0][:, :, :]
        image_lock.release()


if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = torch.hub.load(
        yolo_repo_path, "custom", path=yolo_weights_path, source="local"
    )  # local repo

    rospy.init_node("x_vision_view")
    a = rospy.topics.Subscriber("/camera/color/image_raw", Image, raw_color_callback)
    while not rospy.core.is_shutdown():
        if image_lock.acquire(blocking=False):
            if image is not None:
                cv2.imshow("view", image)
            image_lock.release()
            cv2.waitKey(1)
