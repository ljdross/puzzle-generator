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
        if config["absolute_path_for_meshes_in_urdf"]:
            self._dir_for_output = os.path.abspath(config["dir_for_output"])
        else:
            self._dir_for_output = config["dir_for_output"]
        self.directory = self._dir_for_output + "/" + self.name
        self.urdf_path = self.directory + "/urdf/" + self.name + ".urdf"
        self.link_size_reduction = config["link_size_reduction"]
        self.export_entity_srdf = config["export_entity_srdf"]
        self.export_mesh_dae = config["export_mesh_dae"]
        self.export_mesh_stl = config["export_mesh_stl"]
        self.output_mesh_type = config["output_mesh_type"]
        bpy.context.scene.render.engine = 'BLENDER_WORKBENCH'
        self.base_object = None
        self.floor_thickness = 0
        self.movable_links = []
        self.contains_mesh = False
        self.joint_count = 0

    def update_name(self, new_name="new_default_name"):
        self.name = new_name
        self.directory = self._dir_for_output + "/" + self.name
        self.urdf_path = self.directory + "/urdf/" + self.name + ".urdf"

    def reset(self):
        """Select everything and delete it and reset position of 3D cursor. Also remove data for cameras and meshes."""
        for block in bpy.data.cameras:
            bpy.data.cameras.remove(block)
        for block in bpy.data.meshes:
            bpy.data.meshes.remove(block)
        self.contains_mesh = False
        self.joint_count = 0

        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=True)
        self.movable_links = []
        self.base_object = None

        bpy.context.scene.cursor.location = (0, 0, 0)
        bpy.context.scene.cursor.rotation_euler = (0, 0, 0)

    def subtract_link_size_reduction(self, x):
        if x < self.link_size_reduction:
            return x
        elif x < 2 * self.link_size_reduction:
            return self.link_size_reduction
        else:
            return round(x - self.link_size_reduction, 5)

    def create_visual(self, location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1), material=None, name="",
                      parent=None, mesh="", object_name="", is_cylinder=False):
        """Create a visual object with the appropriate Phobos object properties. Returns the object."""
        scale = tuple(map(self.subtract_link_size_reduction, scale))

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
            visual.scale = scale
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
            visual.name = "visual_box" + name
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

    def create_base_link(self, floor_size=32, thickness=0.2):
        """
        Create a base object to become the base link for all other links.
        If no physical floor is needed, use default floor_size=0
        """
        if floor_size == 0:
            thickness = 0
        self.floor_thickness = thickness
        visual = self.create_visual(scale=(floor_size, floor_size, thickness), material=color.LAVENDER, name="base")
        self.create_link_and_joint(visual, "base_link")
        self.base_object = visual.parent
        if floor_size == 0:
            self.apply_to_subtree(self.base_object, remove_visual=True)
        else:
            self.create_collision(visual)

    def update_joint_axis(self, link, direction_vector=(1, 0, 0)):
        """
        Normally the link moves along (prismatic joint) or rotates around (revolute joint) the Z-axis of the link.
        This axis can be changed.
        E.g. if direction_vector=(1, 0, 0), the link moves along or rotates around the X-axis instead of the Z-axis.
        """
        bpy.ops.object.select_all(action='DESELECT')
        link.data.bones['Bone'].select = True
        bpy.ops.object.mode_set(mode='EDIT')
        bone = bpy.context.selected_bones[0]
        bone.tail = direction_vector
        bpy.ops.object.mode_set(mode='OBJECT')

    def _rename_links_recursively(self, link, link_number, joint_number):
        if not link or not link.parent or link.name == 'base_link':  # exit condition
            return
        if not link.values()[1] == 'fixed':
            new_name = "link_" + str(link_number) + "_joint_" + str(joint_number)
            if link.parent.name == new_name:
                link.parent.name = "link_" + str(link_number) + "_joint_" + str(joint_number + 1)
            link.name = new_name
        self._rename_links_recursively(link.parent, link_number, joint_number + 1)

    def _determine_link_color(self, link_is_child=False):
        num_links = len(self.movable_links)
        if num_links > 1:
            return color.RED
        elif num_links == 0:
            return color.GREEN
        else:  # num_links == 1
            if link_is_child:
                return color.GREEN
            else:
                return color.RED

    def new_link(self, location, rotation, scale, joint_type='fixed', lower_limit=0, upper_limit=0, material=None,
                 auto_limit=0, blend_file="", object_name="", is_cylinder=False, name="", parent=None,
                 create_handle=False, collision=True, joint_axis=(0, 0, 1), new_mesh_name="", hinge_diameter=0):
        if auto_limit != 0:
            if auto_limit < 0:
                lower_limit = auto_limit
            else:
                upper_limit = auto_limit
        if not parent:
            parent = self.base_object
            location = (location[0], location[1], location[2] + self.floor_thickness / 2)
            material = material if material else self._determine_link_color(link_is_child=False)
            link_number = len(self.movable_links)
        else:
            material = material if material else self._determine_link_color(link_is_child=True)
            link_number = len(self.movable_links) - 1
        if joint_type == 'fixed':
            if name:
                name = str(self.joint_count) + "_fixed_" + name
            else:
                name = str(self.joint_count) + "_fixed"
        else:
            name = str(self.joint_count) + "_link_" + str(link_number)
        self.joint_count += 1
        visual = self.create_visual(location, rotation, scale, material, name, parent, blend_file, object_name,
                                    is_cylinder)
        if new_mesh_name:
            visual.data.name = new_mesh_name
        if joint_type != 'fixed':
            self._rename_links_recursively(parent, link_number, joint_number=1)
            name = str(link_number) + "_joint_0"
        self.create_link_and_joint(visual, name, joint_type, lower_limit, upper_limit)
        link = visual.parent
        if scale == (0, 0, 0):
            self.apply_to_subtree(link, remove_visual=True)
        elif collision:
            self.create_collision(visual)
        if joint_type != 'fixed' and parent == self.base_object:
            self.movable_links.append(link)
        if joint_axis != (0, 0, 1):
            # do this (update joint axis) before adding child links e.g. hinge or handle
            self.update_joint_axis(link, joint_axis)
        else:  # joint_axis == (0, 0, 1)
            if joint_type == 'revolute' and hinge_diameter != 0:
                diameter = min(scale[0], scale[1]) * 0.25 if hinge_diameter is None else hinge_diameter
                length = scale[2] * 1.1
                self.new_link((0, 0, 0), (0, 0, 0), (diameter, diameter, length), 'fixed', material=color.GRAY,
                              is_cylinder=True, name="hinge", parent=link, collision=False)
        if create_handle:
            self._create_handle_automatically(link, collision, rotation, scale, joint_type)
        return link

    def new_handle(self, parent, location, rotation=(0, 0, 0), height=1, width=0.2, material=None, is_cylinder=False,
                   collision=True):
        shaft = self.new_link(location, rotation, (width, width, height), 'fixed', material=material,
                              is_cylinder=is_cylinder, name="handle_shaft", parent=parent, collision=collision)
        self.new_link((0, 0, height / 2 + 0.1), (0, 0, 0), (0.05, 0.05, 0.2), 'fixed', material=color.YELLOW,
                      name="handle_knob", parent=shaft, collision=collision)

    def _create_handle_automatically(self, parent, collision, parent_rotation, parent_scale, parent_joint_type,
                                     height=1):
        if parent_joint_type == 'prismatic':
            self.new_handle(parent, (0, 0, parent_scale[2] / 2 + height / 2), (0, 0, 0), height, collision=collision)
        else:
            if parent_scale[0] > parent_scale[1]:
                location = (parent_scale[0] * 0.375, 0, parent_scale[2] / 2 + height / 2)
            else:
                location = (0, parent_scale[1] * 0.375, parent_scale[2] / 2 + height / 2)
            self.new_handle(parent, location, (0, 0, 0), height, is_cylinder=True, collision=collision)

    def new_door(self, location=(0, 0, 1), rotation=(0, 0, 0), scale=(2, 0.2, 2), lower_limit=0, upper_limit=calc.RAD90,
                 cylinder_diameter=0.4, cylinder_material=color.GRAY, handle_material=color.YELLOW, panel_material=None,
                 name="", top_handle=True, collision=True):
        door = self.new_link(location, rotation, (cylinder_diameter, cylinder_diameter, scale[2]), 'revolute',
                             lower_limit, upper_limit, cylinder_material, is_cylinder=True, name=name,
                             collision=collision)
        self.new_link((scale[0] / 2, 0, 0), (0, 0, 0), scale, 'fixed', material=panel_material, name="door_panel",
                      parent=door, collision=collision)
        if top_handle:
            self.new_link((scale[0] * 0.75, 0, scale[2] / 2 + 0.1), (0, 0, 0), (0.2, 0.05, 0.2), 'fixed',
                          material=handle_material, name="door_handle", parent=door, collision=collision)
        else:
            self.new_link((scale[0] * 0.75, scale[1] / 2 + 0.1, 0), (0, 0, 0), (0.05, 0.2, 0.2), 'fixed',
                          material=handle_material, name="door_handle1", parent=door, collision=collision)
            self.new_link((scale[0] * 0.75, -scale[1] / 2 - 0.1, 0), (0, 0, 0), (0.05, 0.2, 0.2), 'fixed',
                          material=handle_material, name="door_handle2", parent=door, collision=collision)
        return door

    def select_with_children(self, object):
        """deselect everything except the object and its children"""
        bpy.context.view_layer.objects.active = object
        bpy.ops.object.select_grouped(extend=False, type='CHILDREN_RECURSIVE')
        object.select_set(True)

    def duplicate_with_children(self, object):
        self.select_with_children(object)
        bpy.ops.object.duplicate()
        return bpy.context.active_object

    def remove_last_object(self):
        self.select_with_children(self.movable_links[-1])
        bpy.ops.object.delete()
        self.movable_links.pop()

    def set_limit_of_latest_link(self, limit, is_prismatic):
        """
        If limit is negative (for revolute joints), lower limit will be set and upper limit will be 0.
        Otherwise, upper limit will be set and lower limit will be 0.
        Appends to self.goal_space and applies self.goal_adjustment.
        """
        if is_prismatic:
            self.movable_links[-1].pose.bones["Bone"].constraints["Limit Location"].max_y = limit
        else:
            if limit < 0:
                self.movable_links[-1].pose.bones["Bone"].constraints["Limit Rotation"].min_x = limit
            else:
                self.movable_links[-1].pose.bones["Bone"].constraints["Limit Rotation"].max_x = limit
        self.export()

    def zeroize_limits(self, link):
        link.pose.bones["Bone"].constraints["Limit Location"].min_x = 0
        link.pose.bones["Bone"].constraints["Limit Location"].min_y = 0
        link.pose.bones["Bone"].constraints["Limit Location"].min_z = 0
        link.pose.bones["Bone"].constraints["Limit Location"].max_x = 0
        link.pose.bones["Bone"].constraints["Limit Location"].max_y = 0
        link.pose.bones["Bone"].constraints["Limit Location"].max_z = 0

        link.pose.bones["Bone"].constraints["Limit Rotation"].use_limit_y = True
        link.pose.bones["Bone"].constraints["Limit Rotation"].use_limit_y = True
        link.pose.bones["Bone"].constraints["Limit Rotation"].use_limit_y = True

        link.pose.bones["Bone"].constraints["Limit Rotation"].min_x = 0
        link.pose.bones["Bone"].constraints["Limit Rotation"].min_y = 0
        link.pose.bones["Bone"].constraints["Limit Rotation"].min_z = 0
        link.pose.bones["Bone"].constraints["Limit Rotation"].max_x = 0
        link.pose.bones["Bone"].constraints["Limit Rotation"].max_y = 0
        link.pose.bones["Bone"].constraints["Limit Rotation"].max_z = 0

    def create_collision(self, visual_obj=None):
        """Create collision objects from visual objects."""
        if visual_obj:
            bpy.ops.object.select_all(action='DESELECT')
            visual_obj.select_set(True)
            shape =  bpy.data.objects[visual_obj.name]['geometry/type']
            bpy.ops.phobos.create_collision_objects(property_colltype=shape)
            collision = bpy.context.selected_objects[0]
            collision.location = visual_obj.location
            return collision
        else:
            bpy.ops.phobos.select_model()
            bpy.ops.phobos.create_collision_objects()

    def apply_to_subtree(self, object, new_material=None, remove_visual=False, remove_collision=False,
                         zeroize_limits=False):
        self.select_with_children(object)
        for obj in bpy.context.selected_objects:
            if obj.phobostype == 'visual':
                if remove_visual:
                    bpy.ops.object.select_all(action='DESELECT')
                    obj.select_set(True)
                    bpy.ops.object.delete()
                elif new_material:
                    obj.active_material = new_material
            elif obj.phobostype == 'collision':
                if remove_collision:
                    bpy.ops.object.select_all(action='DESELECT')
                    obj.select_set(True)
                    bpy.ops.object.delete()
            elif obj.phobostype == 'link':
                if zeroize_limits:
                    self.zeroize_limits(obj)

    def create_goal_duplicate(self, local_translate=(0, 0, 0), rotation_offset=(0, 0, 0),
                              new_material=color.GREEN_TRANSLUCENT):
        goal_duplicate = self.duplicate_with_children(self.movable_links[0])
        self.apply_to_subtree(goal_duplicate, new_material, remove_collision=True, zeroize_limits=True)
        bpy.ops.object.select_all(action='DESELECT')
        goal_duplicate.select_set(True)
        bpy.ops.transform.translate(value=local_translate, orient_type='LOCAL')
        goal_duplicate.rotation_euler = calc.tuple_add(goal_duplicate.rotation_euler, rotation_offset)
        goal_duplicate.name = "goal"
        return goal_duplicate

    def render_image(self, location=(30, 0, 20), rotation=(0.96, 0, calc.RAD90), focal_length=90):
        bpy.ops.object.camera_add(location=location, rotation=rotation)
        cam = bpy.context.object
        cam.data.lens = focal_length + 10
        bpy.context.scene.camera = cam

        bpy.ops.object.select_all(action='SELECT')
        self.base_object.select_set(False)
        for child in self.base_object.children:
            if child.phobostype == 'collision' or child.phobostype == 'visual':
                child.select_set(False)
        bpy.ops.view3d.camera_to_view_selected()

        cam.data.lens = focal_length
        bpy.context.scene.render.filepath = self.directory + "/images/" + self.name + ".png"
        bpy.ops.render.render(write_still=True)
        return cam

    def export(self, add_mesh_filepath_prefix=True, concave_collision_mesh=False):
        """Export model to URDF."""
        bpy.context.scene.phobosexportsettings.path = self.directory
        bpy.context.scene.phobosexportsettings.selectedOnly = False
        bpy.context.scene.export_entity_urdf = True
        bpy.context.scene.export_entity_srdf = self.export_entity_srdf
        if self.contains_mesh:
            bpy.context.scene.export_mesh_dae = self.export_mesh_dae
            bpy.context.scene.export_mesh_stl = self.export_mesh_stl
            bpy.context.scene.phobosexportsettings.outputMeshtype = self.output_mesh_type
        else:
            bpy.context.scene.export_mesh_dae = False
            bpy.context.scene.export_mesh_stl = False
        bpy.ops.object.select_all(action='DESELECT')
        self.base_object.select_set(True)
        bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
        bpy.ops.phobos.name_model(modelname=self.name)
        bpy.ops.phobos.export_model()
        if self.contains_mesh:
            self.modify_urdf(add_mesh_filepath_prefix, concave_collision_mesh)

    def modify_urdf(self, add_mesh_filepath_prefix=True, concave_collision_mesh=False):
        """
        Modify parts of the URDF file after it has been generated by Phobos.

        Args:
            add_mesh_filepath_prefix: Add 'file://' prefix to mesh filepaths created by Phobos.
            concave_collision_mesh: Add 'concave="yes"' attribute to collision meshes in URDF.
                CAUTION: Will disable default convexation in PyBullet and force it to use the actual concave collision
                mesh. However, PyBullet does NOT support collision detection between concave collision meshes!
        """
        with open(self.urdf_path, 'r') as f:
            res = f.read()
            if add_mesh_filepath_prefix:
                res = res.replace('<mesh filename="..', '<mesh filename="file://' + self.directory)
            if concave_collision_mesh:
                res = res.replace('<collision name="collision_mesh', '<collision concave="yes" name="collision_mesh')

        with open(self.urdf_path, 'w') as f:
            f.write(res)
        # Source:
        # https://stackoverflow.com/questions/43875243/find-and-replace-specific-text-within-an-attribute-in-xml-using-python
