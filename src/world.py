import bpy
import os
import sys

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
import color


class BlenderWorld:
    def __init__(self, config):
        """Initialize all attributes with required world properties."""
        if "puzzle_name" in config:
            self.name = config["puzzle_name"]
        else:
            self.name = "default_name"
        self._dir_for_output = config["dir_for_output"]
        self.directory = self._dir_for_output + "/" + self.name
        self.urdf_path = self.directory + "/urdf/" + self.name + ".urdf"
        self.export_entity_srdf = config["export_entity_srdf"]
        self.export_mesh_dae = config["export_mesh_dae"]
        self.base_object = None
        self.movable_objects = []

    def update_name(self, new_name="new_default_name"):
        self.name = new_name
        self.directory = self._dir_for_output + "/" + self.name
        self.urdf_path = self.directory + "/urdf/" + self.name + ".urdf"

    def reset(self):
        """Delete everything and reset position of 3D cursor."""
        self.movable_objects = []
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=True)
        bpy.context.scene.cursor.location = (0, 0, 0)
        bpy.context.scene.cursor.rotation_euler = (0, 0, 0)

    def create_cube(self, name, parent=None, location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1), material=None):
        """Create a visual cube object with the appropriate Phobos object properties. Returns cube object."""
        bpy.ops.mesh.primitive_cube_add(location=location, rotation=rotation, scale=tuple(x / 2 for x in scale))
        cube = bpy.context.active_object
        cube.active_material = material if material else color.WHITE
        bpy.ops.phobos.set_phobostype(phobostype='visual')
        bpy.ops.phobos.define_geometry(geomType='box')
        cube.name = name
        if parent:
            cube.parent = parent
        return cube

    def create_link_and_joint(self, obj, name, joint_type=None, lower=0, upper=0):
        """Create link (at origin of object). Also create joint at child if joint_type is specified."""
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.ops.phobos.create_links(location='selected objects', size=8, parent_link=True, parent_objects=True,
                                    nameformat=name)
        if joint_type:
            bpy.ops.phobos.define_joint_constraints(passive=True, joint_type=joint_type, lower=lower, upper=upper)

    def create_base_link(self, floor_size=0):
        """Create a base object to become the base link for all other links.
        If no physical floor is needed, use default floor_size=0"""
        self.base_object = self.create_cube(name="visual_cube_base", location=(0, 0, -0.1),
                                            scale=(floor_size, floor_size, 0.2), material=color.LAVENDER)
        self.create_link_and_joint(self.base_object, "base_link")

    def new_object(self, location, rotation, scale, joint_type, lower_limit=0, upper_limit=0, material=None):
        if not material:
            if joint_type == 'prismatic':
                material = color.RED
            elif joint_type == 'revolute':
                material = color.GREEN
        i = len(self.movable_objects)
        cube = self.create_cube(name="visual_cube" + str(i), parent=self.base_object, location=location,
                                rotation=rotation, scale=scale, material=material)
        self.movable_objects.append(cube)
        self.create_link_and_joint(cube, "link" + str(i), joint_type=joint_type, lower=lower_limit, upper=upper_limit)

    def remove_last_object(self):
        bpy.ops.object.select_all(action='DESELECT')
        i = str(len(self.movable_objects) - 1)
        bpy.data.objects['collision_cube' + i].select_set(True)
        bpy.data.objects['visual_cube' + i].select_set(True)
        bpy.data.objects['link' + i].select_set(True)
        bpy.ops.object.delete()
        self.movable_objects.pop()

    def set_limit_of_active_object(self, limit, is_prismatic):
        """
        If limit is negative (for revolute joints), lower limit will be set and upper limit will be 0.
        Otherwise, upper limit will be set and lower limit will be 0.
        Appends to self.goal_space and applies self.goal_adjustment.
        """
        if is_prismatic:
            bpy.context.object.pose.bones["Bone"].constraints["Limit Location"].max_y = limit
        else:
            if limit < 0:
                bpy.context.object.pose.bones["Bone"].constraints["Limit Rotation"].min_x = limit
            else:
                bpy.context.object.pose.bones["Bone"].constraints["Limit Rotation"].max_x = limit
        self.export()

    def create_collision(self, obj=None):
        """Create collision objects from visual objects."""
        if obj:
            bpy.ops.object.select_all(action='DESELECT')
            obj.select_set(True)
        else:
            bpy.ops.phobos.select_model()
        bpy.ops.phobos.create_collision_objects()

    def export(self):
        """Export model to URDF."""
        bpy.context.scene.phobosexportsettings.path = self.directory
        bpy.context.scene.phobosexportsettings.selectedOnly = False
        bpy.context.scene.export_entity_urdf = True
        bpy.context.scene.export_entity_srdf = self.export_entity_srdf
        bpy.context.scene.export_mesh_dae = self.export_mesh_dae
        bpy.ops.phobos.name_model(modelname=self.name)
        bpy.ops.phobos.export_model()
