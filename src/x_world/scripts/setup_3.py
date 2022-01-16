#! /usr/bin/env python3

import math
import random
from itertools import product

from world import Spawner
from geometry_msgs.msg import Point, Quaternion

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

x_space = [0.45, 0.60, 0.75]
y_space = [0.15, -0.15]
z = 0.8


if __name__ == "__main__":
    spawner = Spawner()
    random_names = names.copy() + names.copy()
    random.shuffle(random_names)
    random_names = random_names[:len(random_names)//2]
    # random_names = random_names[:1]

    random_positions = list(product(x_space, y_space))
    random.shuffle(random_positions)

    for name, (x, y) in zip(random_names, random_positions):
        qx, qy, qz, qw = [random.random() * 2 * math.pi for _ in range(4)]
        spawner.spawn_model(name, pos=Point(x, y, z), quat=Quaternion(qx, qy, qz, qw))
