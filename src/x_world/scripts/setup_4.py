#! /usr/bin/env python3

import random
from itertools import product

from geometry_msgs.msg import Point, Quaternion

from world import Spawner

block_2x2 = "X2-Y2-Z2"
table = "table_build"

x_space = [0.45, 0.55, 0.65, 0.75]
y_space = [0.20, 0.10, 0.0, -0.10, -0.20]
z = 0.8


if __name__ == "__main__":
    spawner = Spawner()

    random_position = list(product(x_space, y_space))
    random.shuffle(random_position)
    random_position = random_position[:15]

    spawner.spawn_model(table, Point(0, 0.75, 0), Quaternion(0, 0, 0.7068252, 0.7073883))

    for (x, y) in random_position:
        spawner.spawn_model(block_2x2, pos=Point(x, y, z))
