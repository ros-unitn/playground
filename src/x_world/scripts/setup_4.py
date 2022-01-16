#! /usr/bin/env python3

import random
from itertools import product

from geometry_msgs.msg import Point, Quaternion

from world import Spawner

block_2x2 = "X2-Y2-Z2"
block_2x2_fillet = "X2-Y2-Z2-FILLET"

x_space = [0.50, 0.60, 0.70]
y_space = [0.20, 0.10, 0.0, -0.10, -0.20]
z = 0.8


if __name__ == "__main__":
    spawner = Spawner()

    for i, (x, y) in enumerate(product(x_space, y_space)):
        if i == 0:
            spawner.spawn_model(block_2x2_fillet, pos=Point(x, y, z))
        else:
            spawner.spawn_model(block_2x2, pos=Point(x, y, z))
