import bpy
import os
import sys

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
import color
import calc


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
        self.movable_links = []
        self.contains_mesh = False

    def update_name(self, new_name="new_default_name"):
        self.name = new_name
        self.directory = self._dir_for_output + "/" + self.name
        self.urdf_path = self.directory + "/urdf/" + self.name + ".urdf"

    def reset(self):
        """Delete everything and reset position of 3D cursor."""
        self.movable_links = []
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=True)
        bpy.context.scene.cursor.location = (0, 0, 0)
        bpy.context.scene.cursor.rotation_euler = (0, 0, 0)

    def create_visual(self, location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1), material=None, name="",
                      parent=None, mesh="", object_name="", is_cylinder=False):
        """Create a visual object with the appropriate Phobos object properties. Returns the object."""
        if mesh:
            self.contains_mesh = True

            inner_path = 'Object'
            bpy.ops.wm.append(filepath=os.path.join(mesh, inner_path, object_name),
                              directory=os.path.join(mesh, inner_path), filename=object_name)
            # Source:
            # https://b3d.interplanety.org/en/how-to-append-an-object-from-another-blend-file-to-the-scene-using-the-blender-python-api/

            bpy.ops.object.select_all(action='DESELECT')
            bpy.data.objects[object_name].select_set(True)
            bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
            visual = bpy.context.active_object
            visual.location = location
            visual.rotation_euler = rotation
            visual.scale = tuple(x / 2 for x in scale)
            bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        elif is_cylinder:
            bpy.ops.mesh.primitive_cylinder_add(location=location, rotation=rotation, scale=tuple(x / 2 for x in scale))
            visual = bpy.context.active_object
        else:
            bpy.ops.mesh.primitive_cube_add(location=location, rotation=rotation, scale=tuple(x / 2 for x in scale))
            visual = bpy.context.active_object
        visual.active_material = material if material else visual.active_material
        bpy.ops.phobos.set_phobostype(phobostype='visual')
        name = "_" + name if name else ""
        if mesh:
            bpy.ops.phobos.define_geometry(geomType='mesh')
            visual.name = "visual_mesh" + name
        elif is_cylinder:
            bpy.ops.phobos.define_geometry(geomType='cylinder')
            visual.name = "visual_cylinder" + name
        else:
            bpy.ops.phobos.define_geometry(geomType='box')
            visual.name = "visual_cube" + name
        if parent:
            visual.parent = parent
        return visual

    def create_link_and_joint(self, obj, name="", joint_type=None, lower=0, upper=0):
        """Create link (at origin of object). Also create joint at child if joint_type is specified."""
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        name = name if name == "base_link" else "link_" + name
        bpy.ops.phobos.create_links(location='selected objects', size=10, parent_link=True, parent_objects=True,
                                    nameformat=name)
        if joint_type:
            bpy.ops.phobos.define_joint_constraints(passive=True, joint_type=joint_type, lower=lower, upper=upper)

    def create_base_link(self, floor_size=0, thickness=0.2):
        """Create a base object to become the base link for all other links.
        If no physical floor is needed, use default floor_size=0"""
        self.floor_thickness = thickness
        self.base_object = self.create_visual(scale=(floor_size, floor_size, thickness), material=color.LAVENDER,
                                              name="base")
        self.create_link_and_joint(self.base_object, "base_link")
        if floor_size != 0:
            self.create_collision(self.base_object)

    def determine_link_color(self):
        if len(self.movable_links) == 0:
            return color.GREEN
        else:
            return color.RED

    def new_link(self, location, rotation, scale, joint_type, lower_limit=0, upper_limit=0, material=None,
                 mesh_filepath="", object_name="", is_cylinder=False, child_visuals=None, name=""):
        material = material if material else self.determine_link_color()
        i = str(len(self.movable_links))
        name = name + "_" + i if name else i
        visual = self.create_visual(location=(location[0], location[1], location[2] + self.floor_thickness / 2),
                                    rotation=rotation, scale=scale, material=material, name=name,
                                    parent=self.base_object, mesh=mesh_filepath, object_name=object_name,
                                    is_cylinder=is_cylinder)
        for child_visual in (child_visuals or []):
            child_visual.name += "_" + i
            child_visual.active_material = child_visual.active_material if child_visual.active_material else material
            child_visual.parent = visual
            self.create_collision(child_visual)
        self.create_link_and_joint(visual, name=name, joint_type=joint_type, lower=lower_limit, upper=upper_limit)
        self.create_collision(visual)
        self.movable_links.append(visual.parent)
        return visual

    def new_door(self, location=(0, 0, 1), rotation=(0, 0, 0), scale=(2, 0.2, 2), lower_limit=0, upper_limit=calc.RAD90,
                 cylinder_diameter=0.4, cylinder_material=color.GRAY, panel_material=None, child_visuals=None,
                 name="door"):
        panel_material = panel_material if panel_material else self.determine_link_color()
        child_visuals = child_visuals if child_visuals else []
        panel = self.create_visual((scale[0] / 2, 0, 0), (0, 0, 0), scale, panel_material, name + "_panel")
        child_visuals.append(panel)
        handle = self.create_visual((scale[0] * 0.8, scale[1] / 2 + 0.1, 0), (0, 0, 0), (0.2, 0.2, 0.2),
                                    panel_material, name + "_handle")
        child_visuals.append(handle)
        door = self.new_link(location, rotation, (cylinder_diameter, cylinder_diameter, scale[2]), 'revolute',
                             lower_limit, upper_limit, cylinder_material, is_cylinder=True,
                             child_visuals=child_visuals, name=name)
        return door

    def remove_last_object(self):
        bpy.context.view_layer.objects.active = self.movable_links[-1]
        bpy.ops.object.select_grouped(extend=False, type='CHILDREN_RECURSIVE')
        self.movable_links[-1].select_set(True)
        bpy.ops.object.delete()
        self.movable_links.pop()

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

    def create_collision(self, visual_obj=None):
        """Create collision objects from visual objects."""
        if visual_obj:
            bpy.ops.object.select_all(action='DESELECT')
            visual_obj.select_set(True)
        else:
            bpy.ops.phobos.select_model()
        shape =  bpy.data.objects[visual_obj.name]['geometry/type']
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
            res = f.read().replace('<mesh filename="..', '<mesh filename="file://' + self.directory).replace(
                '<collision name="collision_mesh', '<collision concave="yes" name="collision_mesh')

        with open(self.urdf_path, 'w') as f:
            f.write(res)
