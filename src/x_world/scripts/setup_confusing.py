#! /usr/bin/env python3

from itertools import product

from world import Spawner
from geometry_msgs.msg import Point

not_confusing = "X2-Y2-Z2"
confusing = ["X1-Y2-Z1", "X1-Y2-Z2", "X1-Y4-Z1", "X1-Y4-Z2"]

x_space = [0.65, 0.75]
y_space = [0.1, -0.1]
z = 0.72


if __name__ == "__main__":
    spawner = Spawner()

    spawner.spawn_model(not_confusing, pos=Point(0.55, 0, z))
    for name, (x, y) in zip(confusing, product(x_space, y_space)):
        spawner.spawn_model(name, pos=Point(x, y, z))