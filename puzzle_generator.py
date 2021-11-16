import sys, os

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import *

# TODO: simple interface for Bora
# TODO: SRDF, Meshes
# TODO: define goal space instead of goal state in ompl

# output settings and world properties
config = {
    "puzzle_name": "gridworld",
    "dir_for_output": "/home/userone/ba/puzzle-generator/puzzles",
    "number_prismatic_joints": 2,
    "number_revolute_joints": 2,
    "use_floor": True,
    "seed_for_randomness": 3,       # choose None for pseudorandom
    "allow_clockwise": True,        # allow both clockwise and counterclockwise rotating revolute joints
    "export_entity_srdf": True,
    "export_mesh_dae": False
}

# create world according to config
world = World(config)
world.build()

# test model
world.test_with_pybullet_ompl(show_gui=True, allowed_planning_time=5.)
