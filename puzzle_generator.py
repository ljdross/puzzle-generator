from math import radians
import sys, os

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from src.world import *


# set up new world
world = World("simple_sliders", "/home/userone/ba/puzzle-generator/puzzles/simple_sliders", number_prismatic_joints=2)

# create sliders
for i in range(world.number_prismatic_joints):
    if i % 2 == 0:
        world.sliders.append(world.create_cube(name="visual_cube" + str(i), parent=world.floor, location=(i/2, i/-2, 0.1), rotation=(radians(90), 0, 0), scale=(0.2, 0.2, 1.6)))
    else:
        world.sliders.append(world.create_cube(name="visual_cube" + str(i), parent=world.floor, location=((i-1)/2, ((i-1)/-2)-1, 0.1), rotation=(0, radians(90), 0), scale=(0.2, 0.2, 1.6)))

    # create link (at origin of object) and joint at child
    world.create_link_and_joint(world.sliders[i], "link" + str(i), joint_type='prismatic', upper=1)

# create collision objects from visual objects
world.create_collision()

# export model
world.export()

# test model
world.test_with_pybullet_ompl()
