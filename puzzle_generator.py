import os
import sys

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import BlenderWorld
from src.sampling import *
from src.pybullet_simulation import solve
from src import robowflex_simulation
from src import calc

# output settings and world properties
world_config = {
    "dir_for_output": "puzzles",                # both absolute and relative paths are allowed
    "link_shrink": 0.005,                       # reduce the size of every link by this value to avoid touching and
                                                # permanent collision
    "export_entity_srdf": True,
    "absolute_path_for_meshes_in_urdf": True,   # generate an absolute path to reference the output meshes from within
                                                # the urdf if True, else use a relative path
    "export_mesh_dae": False,
    "export_mesh_stl": True,
    "output_mesh_type": 'stl',
}

sampler_config = {
    # this part of the config is always required
    "floor_size": 32,
    "scaling": 1,
    "number_prismatic_joints": 4,
    "number_revolute_joints": 2,
    "attempts": 10,
    "seed_for_randomness": 0,  # choose None for pseudorandom
    "create_handle": True,

    # this part is only required for SimpleSlidersSampler
    "gap": 0.1,     # should be between 0 and 0.9 preferably smaller than 0.5

    # this part is only required for GridWorldSampler
    "allow_clockwise": True,  # allow both clockwise and counterclockwise rotating revolute joints
    "epsilon": 0.1,  # reduce the edge length of every box by epsilon
    "branching_per_revolute_joint": 3,  # must be <= 3
    "branching_factor_target": 6,  # cannot reach > (number_revolute_joints * branching_per_revolute_joint)

    # this part is only required for ContinuousSpaceSampler
    "start_planning_time": 0.1,
    "planning_time_multiplier": 5.,  # apply for sampling of next link+joint after successfully sampling one link+joint
    "first_test_time_multiplier": 1.5,  # during the first test the link+joint is immovable and the puzzle must be
                                        # unsolvable. to make sure that it is really UNsolvable, we must provide more
                                        # planning time than in the second test (where we just check solvability)
    "area_size": 3,
    "upper_limit_prismatic": (2, 4),  # random upper limit will be within this interval, lower limit is always 0
    "upper_limit_revolute": (calc.RAD90, calc.RAD180),  # same here (but there is a 50 % chance for the joint to be
                                                        # clockwise)
    "attempts_per_link": 50,

    # this part is only required for Lockbox2017Sampler and LockboxRandomSampler
    "mesh1": "input-meshes/slot_disc.blend",  # both absolute and relative paths are allowed
    "iterations": 2,
    "slider_length": 1.4,
    "slider_width": 0.2,
    "radius_interval": (1, 3),

    # this part is only required for EscapeRoomSampler and MoveTwiceSampler
    "robot_mesh": "input-meshes/droids.blend",  # both absolute and relative paths are allowed

}

# set up world according to world_config
world = BlenderWorld(world_config)

sampler = SimpleSlidersSampler(sampler_config, world)
sampler.build()
solve(world.urdf_path, sampler.start_state, sampler.goal_space, 1., show_gui=True)

sampler = GridWorldSampler(sampler_config, world)
sampler.build()
solve(world.urdf_path, sampler.start_state, sampler.goal_space, 1., show_gui=True)

# sampler_config["number_prismatic_joints"] = 3
# sampler_config["number_revolute_joints"] = 2
# sampler = ContinuousSpaceSampler(sampler_config, world)
# sampler.build()
# solve(world.urdf_path, sampler.start_state, sampler.goal_space, 1., show_gui=True)

sampler = Lockbox2017Sampler(sampler_config, world)
sampler.build()
solve(world.urdf_path, sampler.start_state, sampler.goal_space, 1., show_gui=True)

sampler = LockboxRandomSampler(sampler_config, world)
sampler.build()
solve(world.urdf_path, sampler.start_state, sampler.goal_space, 1., show_gui=True)

sampler = EscapeRoomSampler(sampler_config, world)
sampler.build()
solve(world.urdf_path, sampler.start_state, sampler.goal_space, 1., show_gui=True)

sampler = MoveTwiceSampler(sampler_config, world)
sampler.build()
solve(world.urdf_path, sampler.start_state, sampler.goal_space, 1., show_gui=True)
