import os
import sys

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import BlenderWorld
from src.sampling import *
from src import pybullet_simulation
from src import robowflex_simulation
from src import calc

# output settings and world properties
world_config = {
    "dir_for_output": "../benchmarking/v0",   # both absolute and relative paths are allowed
    "link_size_reduction": 0.001,               # reduce the size of every link by this value to avoid touching and
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
    "number_prismatic_joints": 0,
    "number_revolute_joints": 0,
    "branching_factor_target": 2,  # should not be higher than number_revolute_joints
    "attempts": 50,
    "seed_for_randomness": 0,  # choose None for pseudorandom
    "create_handle": True,

    # this part is only required for GridWorldSampler
    "allow_clockwise": True,  # allow both clockwise and counterclockwise rotating revolute joints
    "epsilon": 0.1,  # reduce the edge length of every box by epsilon

    # this part is only required for ContinuousSpaceSampler
    "start_planning_time": 0.25,
    "planning_time_multiplier": 2.,  # apply for sampling of next link+joint after successfully sampling one link+joint
    "first_test_time_multiplier": 1.5,  # during the first test the link+joint is immovable and the puzzle must be
                                        # unsolvable. to make sure that it is really UNsolvable, we must provide more
                                        # planning time than in the second test (where we just check solvability)
    "area_size": 3,
    "upper_limit_prismatic": (2, 4),  # random upper limit will be within this interval, lower limit is always 0
    "upper_limit_revolute": (calc.RAD90, calc.RAD180),  # same here (but there is a 50 % chance for the joint to be
                                                        # clockwise)

    # this part is only required for Lockbox2017Sampler and LockboxRandomSampler
    "mesh1": "input-meshes/slot_disc.blend",  # both absolute and relative paths are allowed
    "iterations": 2,

    # this part is only required for EscapeRoomSampler and MoveTwiceSampler
    "robot_mesh": "input-meshes/droids.blend",  # both absolute and relative paths are allowed

}

# set up world according to world_config
world = BlenderWorld(world_config)

BENCHMARK_RUNS = 4
VERSIONS = 1

for i in range(VERSIONS):
    sampler_config["number_prismatic_joints"] = 4
    sampler_config["puzzle_name"] = "simple_sliders_" + str(i)
    sampler = SimpleSlidersSampler(sampler_config, world)
    sampler.build()
    robowflex_simulation.solve(world.urdf_path, planning_time=30., benchmark_runs=BENCHMARK_RUNS)
    # pybullet_simulation.solve(world.urdf_path, sampler.start_state, sampler.goal_space, 10., True)

    sampler_config["number_prismatic_joints"] = 3
    sampler_config["number_revolute_joints"] = 3
    sampler_config["epsilon"] = 0.1
    sampler_config["seed_for_randomness"] = i
    sampler_config["puzzle_name"] = "grid_world_" + str(i)
    sampler = GridWorldSampler(sampler_config, world)
    sampler.build()
    robowflex_simulation.solve(world.urdf_path, planning_time=30., benchmark_runs=BENCHMARK_RUNS)
    # pybullet_simulation.solve(world.urdf_path, sampler.start_state, sampler.goal_space, 10., True)

    sampler_config["number_prismatic_joints"] = 2
    sampler_config["number_revolute_joints"] = 2
    sampler_config["seed_for_randomness"] = i
    sampler_config["puzzle_name"] = "continuous_space_" + str(i)
    sampler = ContinuousSpaceSampler(sampler_config, world)
    sampler.build()
    robowflex_simulation.solve(world.urdf_path, planning_time=0.5, benchmark_runs=BENCHMARK_RUNS)
    # pybullet_simulation.solve(world.urdf_path, sampler.start_state, sampler.goal_space, 10., True)

    sampler_config["epsilon"] = 0.6
    sampler_config["seed_for_randomness"] = i
    sampler_config["puzzle_name"] = "lockbox_random_" + str(i)
    sampler = LockboxRandomSampler(sampler_config, world)
    sampler.build()
    robowflex_simulation.solve(world.urdf_path, planning_time=30., benchmark_runs=BENCHMARK_RUNS)
    # pybullet_simulation.solve(world.urdf_path, sampler.start_state, sampler.goal_space, 10., True)

    # sampler_config["puzzle_name"] = "escape_room_" + str(i)
    # sampler = EscapeRoomSampler(sampler_config, world)
    # sampler.build()
    # robowflex_simulation.solve(world.urdf_path, planning_time=4)

    # sampler_config["puzzle_name"] = "move_twice_" + str(i)
    # sampler = MoveTwiceSampler(sampler_config, world)
    # sampler.build()
    # robowflex_simulation.solve(world.urdf_path, planning_time=1)

    # sampler_config["puzzle_name"] = "lockbox2017_" + str(i)
    # sampler = Lockbox2017Sampler(sampler_config, world)
    # sampler.build()
    # robowflex_simulation.solve(world.urdf_path, planning_time=1)
