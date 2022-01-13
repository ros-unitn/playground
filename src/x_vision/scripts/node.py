#! /usr/bin/env python3

import sys
from threading import Lock
from pathlib import Path

import cv2
import torch
import numpy as np
import rospy
import imutils
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from imutils import perspective as imperspective
from imutils import contours as imcontours

from x_msgs.msg import Block
from x_msgs.srv import Blocks, BlocksResponse

file_path = Path(__file__)
package_path = file_path.parents[1]

yolo_repo_path = Path(package_path, "yolov5")
yolo_weights_path = Path(package_path, "best.pt")

# add yolo to PYTHONPATH
if not yolo_repo_path in sys.path:
    sys.path.append(str(yolo_repo_path))

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


table_depth = 0.6482981

y_min = -1.115
y_max = -0.385

x_min = -0.365
x_max = 0.365

table_gazebo = 0.16
camera_gazebo = 0.65

bridge = CvBridge()

# trained yolo model
model = None

raw_color_lock = Lock()
raw_depth_lock = Lock()
raw_color = None
raw_depth = None


def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)


def detection(raw_color, raw_depth):
    ros_image = np.frombuffer(raw_color.data, dtype=np.uint8).reshape(
        raw_color.height, raw_color.width, -1
    )

    res = model(ros_image)

    color = bridge.imgmsg_to_cv2(raw_color, "bgr8")
    depth = bridge.imgmsg_to_cv2(raw_depth, "32FC1")

    normalized_depth = cv2.normalize(
        depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
    )

    results = []
    for i, row in enumerate(res.xyxy[0]):
        obj = {
            "id": i,
            "label": names[int(row[5])],
            "x1": row[0],
            "y1": row[1],
            "x2": row[2],
            "y2": row[3],
            "confidence": row[4],
        }
        results.append(obj)

    duplicate = []

    for obj in results:
        print(obj)
        for obj2 in results:
            if (
                obj["id"] != obj2["id"]
                and abs(obj["x1"] - obj2["x1"]) < 5
                and abs(obj["y1"] - obj2["y1"]) < 5
                and abs(obj["x2"] - obj2["x2"]) < 5
                and abs(obj["y2"] - obj2["y2"]) < 5
            ):
                if obj["label"] == obj2["label"]:
                    duplicate.append(obj2)
                else:  # two different classifications
                    less_confidence = obj
                    if obj2["confidence"] < obj["confidence"]:
                        less_confidence = obj2
                    duplicate.append(less_confidence)

    # to_remove = list(dict.fromkeys(to_remove)) #do this to remove duplicates, otherwise it will try to remove something that does not exists

    for element in duplicate:
        if element in results:
            results.remove(element)

    message_frame = BlocksResponse()

    for i, obj in enumerate(results):
        y1 = int(obj["y1"])-5
        y2 = int(obj["y2"])+5
        x1 = int(obj["x1"])-5
        x2 = int(obj["x2"])+5

        bbox_depth = cv2.blur(normalized_depth[y1:y2, x1:x2], (1, 1))
        bbox_color = cv2.blur(color[y1:y2, x1:x2], (1, 1))

        surface = bbox_color.copy()

        cv2.imwrite(f"test_depth_{i}.png", bbox_depth)
        cv2.imwrite(f"test_color_{i}.png", bbox_color)

        ## Circle detection

        top_thresh = bbox_depth.min() + 3
        _, top_threshold = cv2.threshold(bbox_depth, top_thresh, 255, cv2.THRESH_BINARY)
        top_img = cv2.blur(top_threshold, (4, 4))
        top_circles = cv2.HoughCircles(
            top_img,
            cv2.HOUGH_GRADIENT,
            1,
            20,
            param1=50,
            param2=30,
            minRadius=0,
            maxRadius=0,
        )

        ## Contours
        edged_threshold = cv2.adaptiveThreshold(
            bbox_depth, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 199, 10
        )
        edged_threshold = cv2.GaussianBlur(edged_threshold, (11, 11), 0)
        edged_img = cv2.Canny(edged_threshold, 50, 100)
        edged_img = cv2.dilate(edged_img, None, iterations=1)
        edged_img = cv2.erode(edged_img, None, iterations=1)  

        contours = cv2.findContours(
            edged_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        contours = imutils.grab_contours(contours)

        ## Angle of the block
        (contours, _) = imcontours.sort_contours(contours)
        contours = sorted(contours, key=lambda c: cv2.contourArea(c))

        assert len(contours) >= 1
        c = contours[-1]

        M = cv2.moments(c)
        assert M["m00"] != 0
        center_x = int(M["m10"] / M["m00"]) + x1
        center_y = int(M["m01"] / M["m00"]) + y1

        x_coord = y_min + (y_max - y_min) * center_y / 1024
        y_coord = x_min + (x_max - x_min) * center_x / 1024

        object_depth = depth[int(center_y), int(center_x)]

        object_p = (object_depth - table_depth) / (0 - table_depth)
        z_coord = table_gazebo + (camera_gazebo - table_gazebo) * object_p

        min_area_rect = cv2.minAreaRect(c)
        angle = min_area_rect[2]
        width = min_area_rect[1][0]
        height = min_area_rect[1][1]
        if width < height:
            angle = angle + 90
        else:
            angle = angle + 180

        ## Conclusions

        point = Point(x_coord, y_coord, z_coord)
        label = obj["label"]
        block = Block()
        block.obj = point
        block.angle = np.radians(angle)
        block.label = label

        message_frame.list.append(block)

        ## Debug

        # cv2.drawContours(
        #     surface, [np.int0(cv2.boxPoints(min_area_rect))], 0, (0, 0, 255), -1
        # )

        # for contour in contours:
        #     approx = cv2.approxPolyDP(
        #         contour, 0.01 * cv2.arcLength(contour, True), True
        #     )
        #     cv2.drawContours(surface, [approx], -1, (0, 255, 0), 3)

        # if top_circles is not None:
        #     circles = np.uint16(np.around(top_circles))
        #     for pt in circles[0, :]:
        #         a, block, r = pt[0], pt[1], pt[2]
        #         cv2.circle(surface, (a, block), r, (255, 0, 0), -1)

        # cv2.circle(
        #     surface, (int(center_x_relative), int(center_y_relative)), 4, (255, 0, 255)
        # )

        # debug = np.concatenate(
        #     (bbox_depth, top_threshold, edged_threshold, edged_img), axis=1
        # )
        # debug = cv2.cvtColor(debug, cv2.COLOR_GRAY2RGB)
        # debug = np.concatenate((debug, surface), axis=1)
        # cv2.imshow(str(i), debug)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

    print(message_frame)
    return message_frame


def raw_color_callback(img: Image):
    if raw_color_lock.acquire(blocking=False):
        global raw_color
        raw_color = img
        raw_color_lock.release()


def raw_depth_callback(img: Image):
    if raw_depth_lock.acquire(blocking=False):
        global raw_depth
        raw_depth = img
        raw_depth_lock.release()


def srv_callback(_):
    global raw_color, raw_depth
    raw_color_lock.acquire()
    raw_depth_lock.acquire()
    result = detection(raw_color, raw_depth)
    raw_depth_lock.release()
    raw_color_lock.release()
    return result


if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = torch.hub.load(
        yolo_repo_path, "custom", path=yolo_weights_path, source="local"
    )  # local repo

    rospy.init_node("x_vision_node")
    a = rospy.topics.Subscriber("/camera/color/image_raw", Image, raw_color_callback)
    b = rospy.topics.Subscriber("/camera/depth/image_raw", Image, raw_depth_callback)
    rospy.rostime.wallsleep(4)
    # srv_callback(1)
    s = rospy.Service("blocks", Blocks, srv_callback)
    rospy.spin()
