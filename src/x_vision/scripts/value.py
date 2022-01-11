#! /usr/bin/env python3

import cv2
import numpy as np
import sys


def callback(x):
    print(x)


assert len(sys.argv) == 2

img = cv2.imread(sys.argv[1], 0)  # read image as grayscale

test_threshold = cv2.adaptiveThreshold(
    img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
)
test_img = cv2.blur(test_threshold, (4, 4))
test_img = cv2.Canny(test_img, 0, 0)
test_img = cv2.dilate(test_img, None, iterations=1)
test_img = cv2.erode(test_img, None, iterations=1)

cv2.namedWindow("image")  # make a window with name 'image'
cv2.createTrackbar(
    "L", "image", 0, 255, callback
)  # lower threshold trackbar for window 'image
cv2.createTrackbar(
    "U", "image", 0, 255, callback
)  # upper threshold trackbar for window 'image

cv2.setTrackbarPos("L", "image", 11)
cv2.setTrackbarPos("U", "image", 2)

while 1:
    numpy_horizontal_concat = np.concatenate(
        (img, test_threshold, test_img), axis=1
    )  # to display image side by side
    cv2.imshow("image", numpy_horizontal_concat)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:  # escape key
        break
    l = cv2.getTrackbarPos("L", "image")
    u = cv2.getTrackbarPos("U", "image")

    if l > 1 and l % 2 == 1:
        print("here")
        test_threshold = cv2.adaptiveThreshold(
            img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, l, u
        )
        test_img = cv2.blur(test_threshold, (4, 4))
        test_img = cv2.Canny(test_img, 0, 0)
        test_img = cv2.dilate(test_img, None, iterations=1)
        test_img = cv2.erode(test_img, None, iterations=1)

cv2.destroyAllWindows()
