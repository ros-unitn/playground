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
camera_gazebo = 0.7314285714285716

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

    depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

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
        y1 = int(obj["y1"])
        y2 = int(obj["y2"])
        x1 = int(obj["x1"])
        x2 = int(obj["x2"])

        bbox_depth = cv2.blur(depth[y1:y2, x1:x2], (1, 1))
        bbox_color = cv2.blur(color[y1:y2, x1:x2], (1, 1))

        ## Circle detection

        up_thresh = bbox_depth.min() + 3
        up_threshold = cv2.threshold(bbox_depth, up_thresh, 255, cv2.THRESH_BINARY)
        up_img = cv2.blur(up_threshold[1], (4, 4))
        up_circles = cv2.HoughCircles(
            up_img,
            cv2.HOUGH_GRADIENT,
            1,
            20,
            param1=50,
            param2=30,
            minRadius=0,
            maxRadius=0,
        )

        ## Find contours
        
        edged = cv2.Canny(bbox_depth, 0, 0)
        edged = cv2.dilate(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)

        contours = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        ## Sides and down

        (contours, _) = imcontours.sort_contours(contours)

        for contour in contours:
            cv2.drawContours(bbox_color, [contour], -1, (0, 255, 0), 2)
            approx = cv2.approxPolyDP(
                contour, 0.05 * cv2.arcLength(contour, True), True)
        
            # Finding center point of shape
            M = cv2.moments(contour)
            if M['m00'] != 0.0:
                x = int(M['m10'] / M['m00'])
                y = int(M['m01'] / M['m00'])
                cv2.putText(bbox_color, str(len(approx)), (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        ## Debug

        if up_circles is not None:
            circles = np.uint16(np.around(up_circles))
            for pt in circles[0, :]:
                a, block, r = pt[0], pt[1], pt[2]
                cv2.circle(bbox_color, (a, block), r, (255, 0, 0), 2)

        cv2.imshow("circle", bbox_color)
        print(len(contours))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        ## Angle
        assert len(contours) >= 1
        c = contours[0]

        box = cv2.minAreaRect(c)
        angle = box[2]
        width = box[1][0]
        height = box[1][1]
        if width < height:
            angle = angle + 90
        else:
            angle = angle + 180

        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")

        box = imperspective.order_points(box)

        (tl, _, br, _) = box

        (center_x, center_y) = midpoint(tl, br)
        center_x = center_x + obj["x1"]
        center_y = center_y + obj["y1"]

        x_coord = y_min + (y_max - y_min) * center_y / 1024
        y_coord = x_min + (x_max - x_min) * center_x / 1024

        object_depth = depth[int(center_y), int(center_x)]
        object_p = (object_depth - table_depth) / (0 - table_depth)

        z_coord = table_gazebo + (camera_gazebo - table_gazebo) * object_p

        point = Point(x_coord, y_coord, z_coord)
        label = obj["label"]
        block = Block()
        block.obj = point
        block.angle = np.radians(angle)
        block.label = label

        message_frame.list.append(block)

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
    rospy.rostime.wallsleep(2)
    srv_callback(1)
    # s = rospy.Service("blocks", Blocks, srv_callback)
    # rospy.spin()
