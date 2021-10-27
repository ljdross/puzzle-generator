import sys, os

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import *

# TODO: config dictionary for world properties
# TODO: simple interface for Bora

# set up new world
world = World("simple_sliders", "/home/userone/ba/puzzle-generator/puzzles/simple_sliders", number_prismatic_joints=2)
world.build()

# test model
world.test_with_pybullet_ompl(show_gui=False, allowed_planning_time=5.)
