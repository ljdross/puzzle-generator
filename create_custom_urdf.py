from math import radians
import os
import sys
DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import *
from src import calc


# output settings (ADJUST AS NEEDED)
config = {
    "puzzle_name": "my_custom_urdf",
    "dir_for_output": "/home/userone/ba/puzzle-generator/puzzles",
    "custom_urdf": True,
    "floor_size": 0,
    "export_entity_srdf": True,
    "export_mesh_dae": False
}

# create world
world = World(config)
world.reset()
world.create_base_link()


# add custom objects (ADJUST AS NEEDED)
world.new_object((0, 5, 2), (0, 0, 0), (2, 0.2, 4), 'revolute', radians(-45), radians(45))

upper_limit = 2
length = 4
angle_rad = radians(30)
world.new_object((0, 0, 0.5), (radians(-90), 0, angle_rad), (1, 1, length), 'prismatic', 0, upper_limit)
joint_end_point = (0, 0 + length / 2 + upper_limit)
world.new_object((joint_end_point[0], joint_end_point[1], 0.1), (0, 0, 0), (0.2, 0.2, 0.2), 'prismatic', 0, 0)
joint_end_point = calc.rotate(joint_end_point, angle_rad)
world.new_object((joint_end_point[0], joint_end_point[1], 0.1), (0, 0, angle_rad), (0.2, 0.2, 0.2), 'prismatic', 0, 0)


# export
world.create_collision()
world.export()

