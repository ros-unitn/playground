#! /usr/bin/env python3

import os
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np

import rospy
from sensor_msgs.msg import Image

file_path = os.path.realpath(__file__)

def image_callback_1(image):
    print(image.height, image.width)
    ros_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
    res = model(ros_image)
    res.render()
    cv2.imshow("Preview", res.imgs[0][:, :, :]*255)
    cv2.waitKey(1)
    print(res.pandas().xyxy[0])

if __name__ == '__main__':
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = torch.hub.load('/home/ros/Workspace/src/ros_yolo/yolov5-master', 'custom', path='/home/ros/Workspace/src/ros_yolo/last.pt', source='local')  # local repo

    rospy.init_node('ros_yolo')
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback_1, queue_size=1, buff_size=52428800)
    
    rospy.spin()
