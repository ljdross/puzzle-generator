import os
import sys

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import BlenderWorld
from src.sampling import SimpleSlidersSampler, ContinuousSpaceSampler, GridWorldSampler
from src.solvability_testing import test_urdf

# output settings and world properties
world_config = {
    "dir_for_output": "/home/userone/ba/puzzle-generator/puzzles",
    "export_entity_srdf": True,
    "export_mesh_dae": False,
}

sampler_config = {
    # this part of the config is always required
    "floor_size": 32,
    "number_prismatic_joints": 1,
    "number_revolute_joints": 2,
    "branching_factor_target": 2,  # should not be higher than number_revolute_joints
    "attempts": 50,
    "seed_for_randomness": 0,  # choose None for pseudorandom

    # this part is only required for GridWorldSampler
    "allow_clockwise": True,  # allow both clockwise and counterclockwise rotating revolute joints

    # this part is only required for ContinuousSpaceSampler

    # this part is only required for ...

}

# set up world according to world_config
world = BlenderWorld(world_config)

sampler = SimpleSlidersSampler(sampler_config, world)
sampler.build()
test_urdf(world.urdf_path, sampler.start_state, sampler.goal_space, show_gui=True)

sampler = GridWorldSampler(sampler_config, world)
sampler.build()
test_urdf(world.urdf_path, sampler.start_state, sampler.goal_space, show_gui=True)

sampler = ContinuousSpaceSampler(sampler_config, world)
sampler.build()
test_urdf(world.urdf_path, sampler.start_state, sampler.goal_space, show_gui=True)
