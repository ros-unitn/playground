#! /usr/bin/env python3

import random
from itertools import product
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Quaternion

from world import Spawner

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

table = "table_drop"

x_space = [0.45, 0.55, 0.65, 0.75]
y_space = [0.2, 0.0, -0.2]
z = 0.72

random_seed = 1642458395.3415635

if __name__ == "__main__":
    spawner = Spawner()
    random.seed(random_seed)
    random_names = names.copy()
    random.shuffle(random_names)

    spawner.spawn_model(table,Point(0,0.75,0),Quaternion(0,0,0.7068252,0.7073883))

    for name, (x, y) in zip(random_names, product(x_space, y_space)):
        spawner.spawn_model(name, pos=Point(x, y, z))
