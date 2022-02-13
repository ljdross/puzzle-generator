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
        self.export_mesh_stl = config["export_mesh_stl"]
        self.output_mesh_type = config["output_mesh_type"]
        self.base_object = None
        self.floor_thickness = 0
        self.movable_visual_objects = []
        self.contains_mesh = False

    def update_name(self, new_name="new_default_name"):
        self.name = new_name
        self.directory = self._dir_for_output + "/" + self.name
        self.urdf_path = self.directory + "/urdf/" + self.name + ".urdf"

    def reset(self):
        """Delete everything and reset position of 3D cursor."""
        self.movable_visual_objects = []
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=True)
        bpy.context.scene.cursor.location = (0, 0, 0)
        bpy.context.scene.cursor.rotation_euler = (0, 0, 0)

    def create_visual(self, name, parent=None, location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1), material=None,
                      mesh="", object_name=""):
        """Create a visual object with the appropriate Phobos object properties. Returns the object."""
        if mesh:
            self.contains_mesh = True

            inner_path = 'Object'
            bpy.ops.wm.append(filepath=os.path.join(mesh, inner_path, object_name),
                              directory=os.path.join(mesh, inner_path), filename=object_name)
            # Source:
            # https://b3d.interplanety.org/en/how-to-append-an-object-from-another-blend-file-to-the-scene-using-the-blender-python-api/

            bpy.ops.object.select_all(action='DESELECT')
            bpy.data.objects['slot_disc'].select_set(True)
            bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
            visual = bpy.context.active_object
            visual.location = location
            visual.rotation_euler = rotation
            visual.scale = tuple(x / 2 for x in scale)
        else:
            bpy.ops.mesh.primitive_cube_add(location=location, rotation=rotation, scale=tuple(x / 2 for x in scale))
            visual = bpy.context.active_object
        visual.active_material = material if material else color.WHITE
        bpy.ops.phobos.set_phobostype(phobostype='visual')
        if mesh:
            bpy.ops.phobos.define_geometry(geomType='mesh')
        else:
            bpy.ops.phobos.define_geometry(geomType='box')
        visual.name = name
        if parent:
            visual.parent = parent
        return visual

    def create_link_and_joint(self, obj, name, joint_type=None, lower=0, upper=0):
        """Create link (at origin of object). Also create joint at child if joint_type is specified."""
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.ops.phobos.create_links(location='selected objects', size=10, parent_link=True, parent_objects=True,
                                    nameformat=name)
        if joint_type:
            bpy.ops.phobos.define_joint_constraints(passive=True, joint_type=joint_type, lower=lower, upper=upper)

    def create_base_link(self, floor_size=0, thickness=0.2):
        """Create a base object to become the base link for all other links.
        If no physical floor is needed, use default floor_size=0"""
        self.floor_thickness = thickness
        self.base_object = self.create_visual(name="visual_cube_base", location=(0, 0, 0),
                                              scale=(floor_size, floor_size, thickness), material=color.LAVENDER)
        self.create_link_and_joint(self.base_object, "base_link")
        if floor_size != 0:
            pass
            # TODO: collision

    def new_object(self, location, rotation, scale, joint_type, lower_limit=0, upper_limit=0, material=None,
                   mesh_filepath="", object_name=""):
        if not material:
            if joint_type == 'prismatic':
                material = color.RED
            elif joint_type == 'revolute':
                material = color.GREEN
        name = "visual_mesh" if mesh_filepath else "visual_cube"
        i = len(self.movable_visual_objects)
        visual = self.create_visual(name=name + str(i), parent=self.base_object,
                                    location=(location[0], location[1], location[2] + self.floor_thickness / 2),
                                    rotation=rotation, scale=scale, material=material, mesh=mesh_filepath,
                                    object_name=object_name)
        self.movable_visual_objects.append(visual)
        self.create_link_and_joint(visual, "link" + str(i), joint_type=joint_type, lower=lower_limit, upper=upper_limit)
        # TODO: collision
        return visual

    def remove_last_object(self):
        bpy.ops.object.select_all(action='DESELECT')
        i = str(len(self.movable_visual_objects) - 1)
        bpy.data.objects['collision_cube' + i].select_set(True)
        bpy.data.objects['visual_cube' + i].select_set(True)
        bpy.data.objects['link' + i].select_set(True)
        bpy.ops.object.delete()
        self.movable_visual_objects.pop()

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

    def create_collision(self, visual_obj=None, shape='box'):
        """Create collision objects from visual objects."""
        if visual_obj:
            bpy.ops.object.select_all(action='DESELECT')
            visual_obj.select_set(True)
        else:
            bpy.ops.phobos.select_model()
        bpy.ops.phobos.create_collision_objects(property_colltype=shape)

    def export(self):
        """Export model to URDF."""
        bpy.context.scene.phobosexportsettings.path = self.directory
        bpy.context.scene.phobosexportsettings.selectedOnly = False
        bpy.context.scene.export_entity_urdf = True
        bpy.context.scene.export_entity_srdf = self.export_entity_srdf
        bpy.context.scene.export_mesh_dae = self.export_mesh_dae
        bpy.context.scene.export_mesh_stl = self.export_mesh_stl
        bpy.context.scene.phobosexportsettings.outputMeshtype = self.output_mesh_type
        bpy.ops.phobos.name_model(modelname=self.name)
        bpy.ops.phobos.export_model()
        if self.contains_mesh:
            self.fix_filepaths_in_urdf()

    def fix_filepaths_in_urdf(self):
        """
        Fix the bad filepaths created by phobos.
        Source:
        https://stackoverflow.com/questions/43875243/find-and-replace-specific-text-within-an-attribute-in-xml-using-python
        """
        with open(self.urdf_path, 'r') as f:
            res = f.read().replace('<mesh filename="..', '<mesh filename="file://' + self.directory)

        with open(self.urdf_path, 'w') as f:
            f.write(res)
