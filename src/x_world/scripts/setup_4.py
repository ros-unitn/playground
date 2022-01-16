#! /usr/bin/env python3

import random
from itertools import product

from geometry_msgs.msg import Point, Quaternion

from world import Spawner

block_2x2 = "X2-Y2-Z2"

x_space = [0.50, 0.60, 0.70]
y_space = [0.20, 0.10, 0.0, -0.10, -0.20]
z = 0.8


if __name__ == "__main__":
    spawner = Spawner()

    random_positions = list(product(x_space, y_space))
    random.shuffle(random_positions)
    random_positions = random_positions[:15]

    for (x, y) in random_positions:
        # qx, qy, qz, qw = [random.random() * 2 * math.pi for _ in range(4)]
        qx, qy, qz, qw = 0, 0, 0, 0
        spawner.spawn_model(block_2x2, pos=Point(x, y, z), quat=Quaternion(qx, qy, qz, qw))
