import os
import sys

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import BlenderWorld
from src import calc

# output settings (ADJUST AS NEEDED)
config = {
    "puzzle_name": "my_custom",
    "dir_for_output": "puzzles",                # both absolute and relative paths are allowed
    "link_size_reduction": 0.001,               # reduce the size of every link by this value to avoid touching and
                                                # permanent collision
    "export_entity_srdf": True,
    "absolute_path_for_meshes_in_urdf": True,   # generate an absolute path to reference the output meshes from within
                                                # the urdf if True, else use a relative path
    "export_mesh_dae": False,
    "export_mesh_stl": True,
    "output_mesh_type": 'stl',
}

# create world
world = BlenderWorld(config)
world.reset()
world.create_base_link(32)

# add custom objects (ADJUST AS NEEDED)
# d2 = world.new_door((4, 4, 2), (0, 0, 0), (2, 0.2, 4))
# d3 = world.new_door((4, -4, 2), (0, 0, 0), (2, 0.2, 4), -calc.RAD90, calc.RAD45, top_handle=False)


first = world.new_link((0, 0, 0.5), (0, 0, 0), (0, 0, 0), 'prismatic', -16, 16, joint_axis=(1, 0, 0))
second = world.new_link((0, 0, 0), (0, 0, 0), (0, 0, 0), 'prismatic', -16, 16, parent=first, joint_axis=(0, 1, 0))
droid = world.new_link((0, 0, 0), (0, 0, 0), (0.75, 1, 1), 'revolute', -calc.RAD180, calc.RAD180, parent=second,
                       mesh_filepath="input-meshes/droids.blend", object_name="droids_3")

sd1 = world.new_link((0, -5, 0.5), (0, 0, 0), (4, 4, 1), 'revolute', -calc.RAD45, calc.RAD45,
                     mesh_filepath="input-meshes/slot_disc.blend", object_name="slot_disc", create_handle=True)
sd2 = world.new_link((0, 5, 0.5), (0, 0, 0), (2, 2, 1), 'revolute', -calc.RAD45, calc.RAD45,
                     mesh_filepath="input-meshes/slot_disc.blend", object_name="slot_disc", create_handle=True)

# export
world.export()
