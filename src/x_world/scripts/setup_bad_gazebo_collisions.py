#! /usr/bin/env python3

from itertools import product
from pathlib import Path

from geometry_msgs.msg import Point, Quaternion

from world import Spawner

block_2x2 = "X2-Y2-Z2"
table = "table_build"

file_path = Path(__file__)
playground_path = file_path.parents[3]
models_path = playground_path.joinpath("models", "blocks.model")

x_space_layer_1 = [0.485, 0.55, 0.615]
y_space_layer_1 = [0.065, 0.0, -0.065]

x_space_layer_2 = [0.5175, 0.5825]
y_space_layer_2 = [0.0325, -0.0325]

table_height = 0.72

buffer_height = 0.01
block_2_height = 0.05 + buffer_height


if __name__ == "__main__":
    spawner = Spawner()
    spawner.spawn_model(table, Point(0, 0.75, 0), Quaternion(0, 0, 0.7068252, 0.7073883))

    for (x, y) in product(x_space_layer_1, y_space_layer_1):
        spawner.spawn_model(block_2x2, pos=Point(x, y, table_height))

    for (x, y) in product(x_space_layer_2, y_space_layer_2):
        spawner.spawn_model(block_2x2, pos=Point(x, y, table_height + block_2_height))