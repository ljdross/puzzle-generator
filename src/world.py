import subprocess
from math import radians
import bpy

class World:
    def __init__(self, config):
        """Initialize all attributes with required world properties."""
        self.name = config["puzzle_name"]
        self.directory = config["dir_for_output"]
        self.number_prismatic_joints = config["number_prismatic_joints"]
        self.number_revolute_joints = config["number_revolute_joints"]
        self.total_number_joints = self.number_prismatic_joints + self.number_revolute_joints
        if config["use_floor"]:
            self.floor_size = 16
        else:
            self.floor_size = 0
        self.base_object = None
        self.movable_objects = []

    def reset(self):
        """Delete everything and reset position of 3D cursor."""
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=True)
        bpy.context.scene.cursor.location = (0, 0, 0)
        bpy.context.scene.cursor.rotation_euler = (0, 0, 0)

    def create_cube(self, name, parent=None, location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
        """Create a visual cube object with the appropriate Phobos object properties."""
        bpy.ops.mesh.primitive_cube_add(location=location, rotation=rotation, scale=tuple(x/2 for x in scale))
        cube = bpy.context.active_object
        bpy.ops.phobos.set_phobostype(phobostype='visual')
        bpy.ops.phobos.define_geometry(geomType='box')
        cube.name = name
        if parent is not None:
            cube.parent = parent
        return cube

    def create_link_and_joint(self, obj, name, joint_type=None, lower=0, upper=0):
        """Create link (at origin of object). Also create joint at child if joint_type is specified."""
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.ops.phobos.create_links(location='selected objects', size=8,
        parent_link=True, parent_objects=True, nameformat=name)
        if joint_type is not None:
            bpy.ops.phobos.define_joint_constraints(passive=True, joint_type=joint_type, lower=lower, upper=upper)
    
    def create_base_link(self):
        """Create a base object to become the base link for all other links.
        If no physical floor is needed, set self.floor_size = 0"""
        self.base_object = self.create_cube(name="visual_cube_base",
        location=(0, 0, -0.1), scale=(self.floor_size, self.floor_size, 0.2))
        self.create_link_and_joint(self.base_object, "base_link")

    def create_simple_sliders(self):
        """Create a very simple model that only works with prismatic joints (number_revolute_joints must equal 0)."""
        for i in range(self.number_prismatic_joints):
            if i % 2 == 0:
                self.movable_objects.append(self.create_cube(name="visual_cube" + str(i), parent=self.base_object,
                location=(i/2, i/-2, 0.1), rotation=(radians(90), 0, 0), scale=(0.2, 0.2, 1.6)))
            else:
                self.movable_objects.append(self.create_cube(name="visual_cube" + str(i), parent=self.base_object,
                location=((i-1)/2, ((i-1)/-2)-1, 0.1), rotation=(0, radians(90), 0), scale=(0.2, 0.2, 1.6)))
            self.create_link_and_joint(self.movable_objects[i], "link" + str(i), joint_type='prismatic', upper=1)

    def create_collision(self):
        """Create collision objects from visual objects."""
        bpy.ops.phobos.select_model()
        bpy.ops.phobos.create_collision_objects()

    def export(self):
        """Export model to URDF."""
        bpy.context.scene.phobosexportsettings.path = self.directory
        bpy.context.scene.phobosexportsettings.selectedOnly = False
        bpy.context.scene.export_entity_urdf = True
        bpy.ops.phobos.name_model(modelname=self.name)
        bpy.ops.phobos.export_model()

    def build(self):
        """Build complete model in Blender and export to URDF."""
        self.reset()
        self.create_base_link()
        self.create_simple_sliders() # TODO: implement a more sophisticated method
        self.create_collision()
        self.export()

    def test_with_pybullet_ompl(self, show_gui=True, allowed_planning_time=5.):
        """Test solvability with [pybullet_ompl](https://github.com/lyf44/pybullet_ompl) as a subprocess."""
        input_path = self.directory + "/urdf/" + self.name + ".urdf"
        start_state = str([0] * (self.total_number_joints))
        goal_state = str([1] * (self.total_number_joints))
        result = subprocess.run(["python3", "pybullet-ompl/pybullet_ompl.py", input_path, start_state, goal_state,
        str(show_gui), str(allowed_planning_time)]).returncode
        if result == 0:
            print("FOUND SOLUTION!")
        else:
            print("DID NOT FIND SOLUTION!")
