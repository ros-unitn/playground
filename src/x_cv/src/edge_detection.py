#!/usr/bin/env python3

import rospy

import cv2
import numpy as np
import imutils
import threading

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from imutils import perspective
from imutils import contours
from x_msg.msg import objectPosition


# Instantiate CvBridge
bridge = CvBridge()

depth_frame = None
depth_frame_lock = threading.Lock()

message_frame = None
message_frame_lock = threading.Lock()

tavolo_depth = 0.78566104

y_min = -1.085345
y_max = -0.253645

x_min = -0.414147
x_max = 0.417553

tavolo_gazebo = 0.152122
camera_gazebo = 0.4354239378211242


def depth_callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    image = cv_image/cv_image.max()*256**2
    image = image.astype(np.uint16)
    with depth_frame_lock:
        global depth_frame
        depth_frame = cv_image


def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)


def coord_depth(x, y):
    with depth_frame_lock:
        if depth_frame is not None:
            return depth_frame[y, x]
    return None


def color_callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    edged = cv2.Canny(gray, 50, 100)
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)

    cnts = cv2.findContours(
        edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    (cnts, _) = contours.sort_contours(cnts)

    image = cv_image.copy()

    with message_frame_lock:
        global message_frame
        message_frame = objectPosition()

        for c in cnts:
            if cv2.contourArea(c) < 100:
                continue

            box = cv2.minAreaRect(c)
            box = cv2.cv.BoxPoints(
                box) if imutils.is_cv2() else cv2.boxPoints(box)
            box = np.array(box, dtype="int")

            box = perspective.order_points(box)

            (tl, _, br, _) = box

            (center_x, center_y) = midpoint(tl, br)

            x_coord = y_min+(y_max-y_min)*center_y/1024
            y_coord = x_min+(x_max-x_min)*center_x/1024

            object_depth = coord_depth(int(center_x), int(center_y))
            object_p = (object_depth-tavolo_depth)/(0-tavolo_depth)

            z_coord = tavolo_gazebo+(camera_gazebo-tavolo_gazebo)*object_p

            actual = Point(x_coord, y_coord, z_coord)

            message_frame.obj.append(actual)


def main():
    rospy.init_node('m_cv')
    depth_topic = "/camera/depth/image_raw"
    color_topic = "/camera/color/image_raw"
    rospy.Subscriber(depth_topic, Image, depth_callback)
    rospy.sleep(1)
    rospy.Subscriber(color_topic, Image, color_callback)
    r = rospy.Rate(1)
    pub = rospy.Publisher('/m_cv/objects', objectPosition, queue_size=10)
    while not rospy.core.is_shutdown():
        with message_frame_lock:
            if message_frame is not None:
                rospy.loginfo(message_frame)
                pub.publish(message_frame)
                r.sleep()

        rospy.rostime.wallsleep(0.5)


if __name__ == '__main__':
    main()
