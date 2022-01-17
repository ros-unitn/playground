#! /usr/bin/env python3

import random

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

table = "table_drop"

if __name__ == "__main__":
    spawner = Spawner()
    name = random.choice(names)
    spawner.spawn_model(table,Point(0,0.75,0),Quaternion(0,0,0.7068252,0.7073883))
    spawner.spawn_model(name)
