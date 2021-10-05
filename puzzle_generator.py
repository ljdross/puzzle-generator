import bpy
from math import radians


def create_cube(name="cube", location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
	bpy.ops.mesh.primitive_cube_add(location=location, rotation=rotation, scale=scale)
	cube = bpy.context.active_object
	bpy.ops.phobos.set_phobostype(phobostype='visual')
	bpy.ops.phobos.define_geometry(geomType='box')
	cube.name = name
	return cube



# delete everything
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=True)
bpy.context.scene.cursor.location = (0, 0, 0)
bpy.context.scene.cursor.rotation_euler = (0, 0, 0)


# create first cube
visual_cube1 = create_cube(name="visual_cube1", location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1))


# create second cube
visual_cube2 = create_cube(name="visual_cube2", location=(0, 0, 2), rotation=(0, 0, radians(45)), scale=(1, 2, 1))


# parent-child relationship
visual_cube2.parent = visual_cube1


# select first object
bpy.ops.object.select_all(action='DESELECT')
bpy.data.objects['visual_cube1'].select_set(True)
#bpy.context.scene.objects.active = visual_cube1

# create link at origin
bpy.ops.phobos.create_links(location='selected objects', size=8, parent_link=True, parent_objects=True, nameformat="base_link")

# select
bpy.ops.object.select_all(action='DESELECT')
bpy.data.objects['visual_cube2'].select_set(True)
# create link at origin
bpy.ops.phobos.create_links(location='selected objects', size=8, parent_link=True, parent_objects=True, nameformat="link1")

# define joint constraint at child link
bpy.ops.phobos.define_joint_constraints(passive=True, useRadian=False, joint_type='revolute', lower=0, upper=90)

# create collision objects from visual objects
bpy.ops.phobos.select_model()
bpy.ops.phobos.create_collision_objects()


# export model
bpy.context.scene.phobosexportsettings.path = "/home/userone/ba/puzzle-generator/puzzles/simple_two_cubes"
bpy.context.scene.phobosexportsettings.selectedOnly = False
bpy.context.scene.export_entity_urdf = True
bpy.ops.phobos.name_model(modelname="simple_two_cubes")
bpy.ops.phobos.export_model(modelname='simple_two_cubes')

