import os
import sys

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import BlenderWorld
from src import calc

# output settings (ADJUST AS NEEDED)
config = {
    "puzzle_name": "my_custom",
    "dir_for_output": "/home/userone/ba/puzzle-generator/puzzles",
    "export_entity_srdf": True,
    "export_mesh_dae": False
}

# create world
world = BlenderWorld(config)
world.reset()
world.create_base_link(32)

# add custom objects (ADJUST AS NEEDED)
world.new_object((0, 5, 2), (0, 0, 0), (2, 0.2, 4), 'revolute', -calc.RAD45, calc.RAD45)

upper_limit = 2
length = 4
angle_rad = calc.RAD45
world.new_object((0, 0, 0.5), (-calc.RAD90, 0, angle_rad), (1, 1, length), 'prismatic', 0, upper_limit)
joint_end_point = (0, 0 + length / 2 + upper_limit)
world.new_object((joint_end_point[0], joint_end_point[1], 0.1), (0, 0, 0), (0.2, 0.2, 0.2), 'prismatic', 0, 0)
joint_end_point = calc.rotate(joint_end_point, angle_rad)
world.new_object((joint_end_point[0], joint_end_point[1], 0.1), (0, 0, angle_rad), (0.2, 0.2, 0.2), 'prismatic', 0, 0)

# export
world.create_collision()

sd = world.new_object((0, -5, 0.5), (0, 0, 0), (1, 1, 1), 'revolute', -calc.RAD45, calc.RAD45,
                      mesh_filepath="/home/userone/ba/blend_files/slot_disc.dae")
world.create_collision(sd, 'mesh')

world.export()
