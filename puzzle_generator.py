from math import radians
from sys import stdin
import bpy
import subprocess


DIR_FOR_OUTPUT = "/home/userone/ba/puzzle-generator/puzzles/simple_sliders"
MODELNAME = "simple_sliders"
NUMBER_OF_SLIDERS = 2


def reset():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=True)
    bpy.context.scene.cursor.location = (0, 0, 0)
    bpy.context.scene.cursor.rotation_euler = (0, 0, 0)

def create_cube(name, parent=None, location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
    bpy.ops.mesh.primitive_cube_add(location=location, rotation=rotation, scale=scale)
    cube = bpy.context.active_object
    bpy.ops.phobos.set_phobostype(phobostype='visual')
    bpy.ops.phobos.define_geometry(geomType='box')
    cube.name = name
    if parent is not None:
        cube.parent = parent
    return cube

def create_link_and_joint(obj, name, joint_type=None, lower=0, upper=0):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.ops.phobos.create_links(location='selected objects', size=8, parent_link=True, parent_objects=True, nameformat=name)
    if joint_type is not None:
        bpy.ops.phobos.define_joint_constraints(passive=True, joint_type=joint_type, lower=lower, upper=upper)

def create_collision():
    bpy.ops.phobos.select_model()
    bpy.ops.phobos.create_collision_objects()

def export(directory, filename):
    bpy.context.scene.phobosexportsettings.path = directory
    bpy.context.scene.phobosexportsettings.selectedOnly = False
    bpy.context.scene.export_entity_urdf = True
    bpy.ops.phobos.name_model(modelname=filename)
    bpy.ops.phobos.export_model()



# delete everything
reset()


# create floor
floor = create_cube(name="floor", location=(0, 0, -0.1), scale=(8, 8, 0.1))
create_link_and_joint(floor, "base_link")

# create sliders
sliders = []
for i in range(NUMBER_OF_SLIDERS):
    if i % 2 == 0:
        sliders.append(create_cube(name="visual_cube" + str(i), parent=floor, location=(i/2, i/-2, 0.1), rotation=(radians(90), 0, 0), scale=(0.1, 0.1, 0.8)))
    else:
        sliders.append(create_cube(name="visual_cube" + str(i), parent=floor, location=((i-1)/2, ((i-1)/-2)-1, 0.1), rotation=(0, radians(90), 0), scale=(0.1, 0.1, 0.8)))

    # create link (at origin of object) and joint at child
    create_link_and_joint(sliders[i], "link" + str(i), joint_type='prismatic', upper=1)


# create collision objects from visual objects
create_collision()


# export model
export(DIR_FOR_OUTPUT, MODELNAME)

input = DIR_FOR_OUTPUT + "/urdf/" + MODELNAME + ".urdf"
if not subprocess.run(["python3", "pybullet-ompl/pybullet_ompl.py", input]).returncode:
    print("FOUND SOLUTION!")
else:
    print("DID NOT FIND SOLUTION!")
