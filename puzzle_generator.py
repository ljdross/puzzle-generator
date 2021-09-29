import bpy
from math import radians

# delete everything
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=True)
bpy.context.scene.cursor_location[0] = 0
bpy.context.scene.cursor_location[1] = 0
bpy.context.scene.cursor_location[2] = 0


# create first cube
bpy.ops.mesh.primitive_cube_add()
visual_cube1 = bpy.context.active_object
bpy.ops.phobos.set_phobostype(phobostype='visual')
bpy.ops.phobos.define_geometry(geomType='box')
visual_cube1.name = "visual_cube1"


# create second cube
bpy.ops.mesh.primitive_cube_add(location=(0, 0, 2))
visual_cube2 = bpy.context.active_object
bpy.ops.phobos.set_phobostype(phobostype='visual')
bpy.ops.phobos.define_geometry(geomType='box')
visual_cube2.name = "visual_cube2"

# parent-child relationship
visual_cube2.parent = visual_cube1


# select first object
bpy.ops.object.select_all(action='DESELECT')
bpy.data.objects['visual_cube1'].select = True
#bpy.context.scene.objects.active = visual_cube1

# create link at origin
bpy.ops.phobos.create_links(location='selected objects', size=8, parent_link=True, parent_objects=True, nameformat="base_link")

# select
bpy.ops.object.select_all(action='DESELECT')
bpy.data.objects['visual_cube2'].select = True
# create link at origin
bpy.ops.phobos.create_links(location='selected objects', size=8, parent_link=True, parent_objects=True, nameformat="link1")

# define joint constraint at child link
bpy.ops.phobos.define_joint_constraints(passive=True, useRadian=False, joint_type='revolute', upper=135)


# export model
bpy.context.scene.phobosexportsettings.path = "/home/userone/ba/puzzles/p8/"
bpy.context.scene.phobosexportsettings.selectedOnly = False
bpy.context.scene.export_entity_urdf = True
bpy.ops.phobos.name_model(modelname="simple_two_cubes")
bpy.ops.phobos.export_model(modelname='simple_two_cubes')
