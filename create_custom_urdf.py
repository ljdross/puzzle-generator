from math import radians
import os
import sys
DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import *


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
world.new_object((0, 0, 0.5), (radians(90), 0, 0), (1, 1, 4), 'prismatic', 0, 2)
world.new_object((0, 4, 2), (0, 0, 0), (2, 0.2, 4), 'revolute', radians(-45), radians(45))

# export
world.create_collision()
world.export()

