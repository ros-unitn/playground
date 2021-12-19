#!/usr/bin/env python3

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from imutils import perspective
from imutils import contours
# OpenCV2 for saving an image
import cv2
import numpy as np
import imutils

# Instantiate CvBridge
bridge = CvBridge()


def depth_callback(msg):
    print("Received an image!")
    cv_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    img_norm = cv_image/cv_image.max()*256**2
    img_norm = img_norm.astype(np.uint16)
    cv2.resize(img_norm, (1024, 1024))
    cv2.imshow("Depth", img_norm)
    cv2.waitKey()
    time = msg.header.stamp
    cv2.imwrite('/home/ros/catkin_ws/src/m_depth/depth/' +
                str(time)+'.png', img_norm)
    rospy.sleep(1)


def depth_array(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    max_h = cv_image.max()
    min_h = cv_image.min()
    print("max_h: ", max_h, " min_h: ", min_h)
    cv2.waitKey()


def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)


def color_callback(msg):
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    edged = cv2.Canny(gray, 50, 100)
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)

    cnts = cv2.findContours(
        edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    (cnts, _) = contours.sort_contours(cnts)
    pixelsPerMetric = None

    for c in cnts:
        if cv2.contourArea(c) < 100:
            continue

        orig = image.copy()
        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")

        box = perspective.order_points(box)
        cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)

        for (x, y) in box:
            cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)

        (tl, tr, br, bl) = box
        (tltrX, tltrY) = midpoint(tl, tr)
        (blbrX, blbrY) = midpoint(bl, br)

        (tlblX, tlblY) = midpoint(tl, bl)
        (trbrX, trbrY) = midpoint(tr, br)

        cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)

        cv2.line(orig, (int(tltrX), int(tltrY)),
                 (int(blbrX), int(blbrY)), (255, 0, 255), 2)
        cv2.line(orig, (int(tlblX), int(tlblY)),
                 (int(trbrX), int(trbrY)), (255, 0, 255), 2)

        render = cv2.resize(orig, (512, 512))
        cv2.imshow("Color", render)
        cv2.waitKey(0)


def main():
    rospy.init_node('m_depth')
    depth_topic = "/camera/depth/image_raw"
    # rospy.Subscriber(depth_topic, Image, depth_callback)
    color_topic = "/camera/color/image_raw"
    rospy.Subscriber(color_topic, Image, color_callback)
    #add relative location of image's center
    rospy.spin()


if __name__ == '__main__':
    main()
