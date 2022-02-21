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
    "export_mesh_dae": False,
    "export_mesh_stl": True,
    "output_mesh_type": 'stl',
}

# create world
world = BlenderWorld(config)
world.reset()
world.create_base_link(32)

# add custom objects (ADJUST AS NEEDED)
# create child visuals first
child_visuals = []
child_visuals.append(world.create_visual((1, 0, 0), (0, 0, 0), (2, 1, 2)))
# then create the parent visual with joint properties and provide list of child visuals
door = world.new_link((0, 5, 1), (0, 0, 0), (1, 1, 2), 'revolute', -calc.RAD45, calc.RAD45, is_cylinder=True,
                      child_visuals=child_visuals)

upper_limit = 2
length = 4
angle_rad = calc.RAD45
world.new_link((0, 0, 0.5), (-calc.RAD90, 0, angle_rad), (1, 1, length), 'prismatic', 0, upper_limit)
# joint_end_point = (0, 0 + length / 2 + upper_limit)
# world.new_link((joint_end_point[0], joint_end_point[1], 0.1), (0, 0, 0), (0.2, 0.2, 0.2), 'prismatic', 0, 0)
# joint_end_point = calc.rotate(joint_end_point, angle_rad)
# world.new_link((joint_end_point[0], joint_end_point[1], 0.1), (0, 0, angle_rad), (0.2, 0.2, 0.2), 'prismatic', 0, 0)

# export

sd = world.new_link((0, -5, 0.5), (0, 0, 0), (4, 4, 1), 'revolute', -calc.RAD45, calc.RAD45,
                    mesh_filepath="/home/userone/ba/puzzle-generator/input-meshes/slot_disc.blend",
                    object_name="slot_disc")

world.export()
