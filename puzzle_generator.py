import os
import sys

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import BlenderWorld
from src.sampling import SimpleSlidersSampler, ContinuousSpaceSampler, GridWorldSampler
from src.solvability_testing import test_urdf
from src import calc

# output settings and world properties
world_config = {
    "dir_for_output": "/home/userone/ba/puzzle-generator/puzzles",
    "export_entity_srdf": True,
    "export_mesh_dae": False,
    "export_mesh_stl": True,
    "output_mesh_type": 'stl',
}

sampler_config = {
    # this part of the config is always required
    "floor_size": 32,
    "number_prismatic_joints": 2,
    "number_revolute_joints": 2,
    "branching_factor_target": 2,  # should not be higher than number_revolute_joints
    "attempts": 50,
    "seed_for_randomness": 0,  # choose None for pseudorandom

    # this part is only required for GridWorldSampler
    "allow_clockwise": True,  # allow both clockwise and counterclockwise rotating revolute joints
    "epsilon": 0.1,  # reduce the edge length of every box by epsilon

    # this part is only required for ContinuousSpaceSampler
    "start_planning_time": 0.1,
    "planning_time_multiplier": 2.,  # apply for sampling of next link+joint after successfully sampling one link+joint
    "first_test_time_multiplier": 1.5,  # during the first test the link+joint is immovable and the puzzle must be
                                        # unsolvable. to make sure that it is really UNsolvable, we must provide more
                                        # planning time than in the second test (where we just check solvability)
    "area_size": 3,
    "upper_limit_prismatic": (2, 4),  # random upper limit will be within this interval, lower limit is always 0
    "upper_limit_revolute": (calc.RAD90, calc.RAD180),  # same here (but there is a 50 % chance for the joint to be
                                                        # clockwise)

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
