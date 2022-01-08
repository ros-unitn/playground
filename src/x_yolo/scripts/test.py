#! /usr/bin/env python3

import sys
from pathlib import Path

from PIL import Image

import cv2
import torch
import numpy as np

file_path = Path(__file__)
package_path = file_path.parents[1]

yolo_repo_path = Path(package_path, "yolov5")
yolo_weights_path = Path(package_path, "best.pt")

# add yolo to PYTHONPATH
if not yolo_repo_path in sys.path:
    sys.path.append(str(yolo_repo_path))

if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = torch.hub.load(
        yolo_repo_path, "custom", path=yolo_weights_path, source="local"
    )  # local repo

    with Image.open("/home/rossi/dev/uni/playground/test_2.png") as f:
        ros_image = np.asarray(f)

        for _ in range(3):
            res = model(ros_image)
        res.show()
