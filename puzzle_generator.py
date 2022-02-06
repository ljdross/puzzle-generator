import os
import sys
DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import World
from src.sampling import SimpleSlidersSampler, ContinuousSpaceSampler, GridWorldSampler
from src.solvability_testing import test_urdf

# output settings and world properties
config = {
    "puzzle_name": "sampleworld",
    "dir_for_output": "/home/userone/ba/puzzle-generator/puzzles",
    "custom_urdf": False,
    "number_prismatic_joints": 1,
    "number_revolute_joints": 2,
    "branching_factor_target": 2,   # should not be higher than number_revolute_joints
    "floor_size": 32,
    "seed_for_randomness": 0,       # choose None for pseudorandom
    "allow_clockwise": True,        # allow both clockwise and counterclockwise rotating revolute joints
    "export_entity_srdf": True,
    "export_mesh_dae": False
}

# create world according to config
world = World(config)

sampler = SimpleSlidersSampler(config, world)
sampler.build()
test_urdf(world.urdf_path, sampler.start_state, sampler.goal_space, show_gui=True)


sampler = GridWorldSampler(config, world)
sampler.build()
test_urdf(world.urdf_path, sampler.start_state, sampler.goal_space, show_gui=True)


sampler = ContinuousSpaceSampler(config, world)
sampler.build()

# test solvability
test_urdf(world.urdf_path, sampler.start_state, sampler.goal_space, show_gui=True)
