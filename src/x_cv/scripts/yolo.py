#! /usr/bin/env python3

import sys

import cv2
import torch
import numpy as np

from pathlib import Path

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import imutils
from cv_bridge import CvBridge
from imutils import perspective
from imutils import contours
from x_msgs.srv import objectCall, objectCallResponse
import threading

file_path = Path(__file__)
package_path = file_path.parents[1]

yolo_repo_path = Path(package_path, "yolov5")
yolo_weights_path = Path(package_path, "best.pt")

names = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET',
         'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']

         
tavolo_depth = 0.78566104

y_min = -1.165345
y_max = -0.253645

x_min = -0.417553
x_max = 0.417553

tavolo_gazebo = 0.152122
camera_gazebo = 0.4354239378211242

# Instantiate CvBridge
bridge = CvBridge()

#use this to unsubscribe from topics
img_dep = None
message_frame = None
#print(yolo_repo_path)
#print(yolo_weights_path)

if not yolo_repo_path in sys.path:
    sys.path.append(str(yolo_repo_path))

def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

def detection(image):
    with sem_color:
        print("callback color")

        ros_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        res = model(ros_image)
        res.render()
        #cv2.imshow("Preview", res.imgs[0][:, :, :]*255)
        #cv2.waitKey(0)

        cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        results = []
        for row in res.xyxy[0]:
            obj = {"label": names[int(row[5])],
                    "x1":  row[0],
                    "y1": row[1],
                    "x2": row[2],
                    "y2": row[3],
                    "confidence": row[4]}
            results.append(obj)

        to_remove = []

        for obj in results:
            for obj2 in results:
                if obj["label"] != obj2["label"]:
                    if abs(obj["x1"] - obj2["x1"]) < 5 and abs(obj["y1"] - obj2["y1"]) < 5 \
                        and abs(obj["x2"] - obj2["x2"]) < 5 and abs(obj["y2"] - obj2["y2"]) < 5:
                        less_confidence = obj
                        if obj2["confidence"] < obj["confidence"]:
                            less_confidence = obj2
                        to_remove.append(less_confidence)
        
        #to_remove = list(dict.fromkeys(to_remove)) #do this to remove duplicates, otherwise it will try to remove something that does not exists

        for element in to_remove:
            if element in results:
                results.remove(element)

        global message_frame
        message_frame = objectCallResponse()

        for obj in results:
            y1 = int(obj["y1"])
            y2 = int(obj["y2"])
            x1 = int(obj["x1"])
            x2 = int(obj["x2"])

            bbox_img = gray[y1:y2, x1:x2]
            edged = cv2.Canny(bbox_img, 50, 100)
            edged = cv2.dilate(edged, None, iterations=1)
            edged = cv2.erode(edged, None, iterations=1)

            cnts = cv2.findContours(
                edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)

            (cnts, _) = contours.sort_contours(cnts)

            for c in cnts:
                if cv2.contourArea(c) < 100:
                    continue

                box = cv2.minAreaRect(c)
                #print(rot)

                box = cv2.cv.BoxPoints(
                    box) if imutils.is_cv2() else cv2.boxPoints(box)
                box = np.array(box, dtype="int")

                box = perspective.order_points(box)

                (tl, _, br, _) = box

                (center_x, center_y) = midpoint(tl, br)
                center_x = center_x + obj["x1"]
                center_y = center_y + obj["y1"]

                cv2.circle(cv_image, (int(center_x), int(center_y)),
                        5, (0, 0, 255), -1)

                x_coord = y_min+(y_max-y_min)*center_y/1024
                y_coord = x_min+(x_max-x_min)*center_x/1024

                #ora voglio la profondità
                #img_dep_raw = rospy.wait_for_message("/camera/depth/image_raw", Image)
                #img_dep = bridge.imgmsg_to_cv2(img_dep_raw, "32FC1")

                object_depth = img_dep[int(center_y), int(center_x)]
                object_p = (object_depth-tavolo_depth)/(0-tavolo_depth)

                z_coord = tavolo_gazebo+(camera_gazebo-tavolo_gazebo)*object_p

                actual = Point(x_coord, y_coord, z_coord)
                message_frame.obj.append(actual)
                #print(actual)

        cv2.imshow("Image", cv_image)
        cv2.waitKey()

        print(message_frame)
        sem_return.release()


def depth_callback(img_dep_raw):
    print("callback depth")
    global img_dep
    img_dep = bridge.imgmsg_to_cv2(img_dep_raw, "32FC1")
    sem_color.release()
    
def srv_callback(req):
    print("service request received")
    global sem_color
    global sem_return
    sem_color = threading.Semaphore(1)
    sem_color.acquire()
    sem_return = threading.Semaphore(1)
    sem_return.acquire()
    depth_s = rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback)
    color_s = rospy.Subscriber("/camera/color/image_raw", Image, detection)
    with sem_return:
        depth_s.unregister()
        color_s.unregister()
        return message_frame

if __name__ == '__main__':
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = torch.hub.load(yolo_repo_path, 'custom', path=yolo_weights_path, source='local')  # local repo

    rospy.init_node('ros_yolo')
    s = rospy.Service('blocks', objectCall, srv_callback)
    while not rospy.core.is_shutdown():
        pass
