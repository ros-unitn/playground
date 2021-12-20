#!/usr/bin/env python3

import rospy

import cv2
import numpy as np
import imutils
import threading

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from imutils import perspective
from imutils import contours


# Instantiate CvBridge
bridge = CvBridge()

depth_frame = None
depth_time = None
depth_frame_lock = threading.Lock()

color_frame = None
color_time = None
color_frame_lock = threading.Lock()


def depth_callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    image = cv_image/cv_image.max()*256**2
    image = image.astype(np.uint16)
    cv2.resize(image, (1024, 1024))
    with depth_frame_lock:
        global depth_frame, depth_time
        depth_frame = image
        depth_time = msg.header.stamp
    rospy.sleep(1)


def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)


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

    for c in cnts:
        if cv2.contourArea(c) < 100:
            continue

        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")

        box = perspective.order_points(box)

        cv2.drawContours(image, [box.astype("int")], -1, (0, 255, 0), 2)

        # for (x, y) in box:
        #    cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)

        (tl, tr, _, bl) = box
        tltr_x = midpoint(tl, tr)[0]

        tlbl_y = midpoint(tl, bl)[1]

        (center_x, center_y) = (tltr_x, tlbl_y)

        y_min = -1.085345
        y_max = -0.253645

        x_min = -0.414147
        x_max = 0.417553

        cXp = x_min+(x_max-x_min)*center_x/1024
        cYp = y_min+(y_max-y_min)*center_y/1024

        cv2.putText(image, "x: {:.6f}".format(cYp),
                    (int(center_x + 10), int(center_y - 50)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2)
        cv2.putText(image, "y: {:.6f}".format(cXp),
                    (int(center_x + 50), int(center_y - 20)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2)

        cv2.circle(image, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)

    with color_frame_lock:
        global color_frame
        color_frame = image


def main():
    rospy.init_node('m_depth')
    depth_topic = "/camera/depth/image_raw"
    color_topic = "/camera/color/image_raw"
    cv2.namedWindow("depth", cv2.WINDOW_NORMAL)
    cv2.namedWindow("color", cv2.WINDOW_NORMAL)
    rospy.Subscriber(depth_topic, Image, depth_callback)
    rospy.Subscriber(color_topic, Image, color_callback)
    while not rospy.core.is_shutdown():
        with depth_frame_lock:
            if depth_frame is not None:
                cv2.imshow('depth', depth_frame)
        with color_frame_lock:
            if color_frame is not None:
                cv2.imshow('color', color_frame)
        cv2.waitKey(1)
        rospy.rostime.wallsleep(0.5)


if __name__ == '__main__':
    main()
