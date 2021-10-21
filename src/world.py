import subprocess
import bpy

class World:
    def __init__(self, name, directory, number_prismatic_joints=0, number_revolute_joints=0):
        self.name = name
        self.directory = directory
        self.number_prismatic_joints = number_prismatic_joints
        self.number_revolute_joints = number_revolute_joints

        # delete everything
        self.reset()

        # create floor
        self.floor = self.create_cube(name="floor", location=(0, 0, -0.1), scale=(16, 16, 0.1))
        self.create_link_and_joint(self.floor, "base_link")

        self.sliders = []

    def reset(self):
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=True)
        bpy.context.scene.cursor.location = (0, 0, 0)
        bpy.context.scene.cursor.rotation_euler = (0, 0, 0)

    def create_cube(self, name, parent=None, location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
        bpy.ops.mesh.primitive_cube_add(location=location, rotation=rotation, scale=tuple(x/2 for x in scale))
        cube = bpy.context.active_object
        bpy.ops.phobos.set_phobostype(phobostype='visual')
        bpy.ops.phobos.define_geometry(geomType='box')
        cube.name = name
        if parent is not None:
            cube.parent = parent
        return cube

    def create_link_and_joint(self, obj, name, joint_type=None, lower=0, upper=0):
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.ops.phobos.create_links(location='selected objects', size=8, parent_link=True, parent_objects=True, nameformat=name)
        if joint_type is not None:
            bpy.ops.phobos.define_joint_constraints(passive=True, joint_type=joint_type, lower=lower, upper=upper)

    def create_collision(self):
        bpy.ops.phobos.select_model()
        bpy.ops.phobos.create_collision_objects()

    def export(self):
        bpy.context.scene.phobosexportsettings.path = self.directory
        bpy.context.scene.phobosexportsettings.selectedOnly = False
        bpy.context.scene.export_entity_urdf = True
        bpy.ops.phobos.name_model(modelname=self.name)
        bpy.ops.phobos.export_model()

    def test_with_pybullet_ompl(self):
        input = self.directory + "/urdf/" + self.name + ".urdf"
        if not subprocess.run(["python3", "pybullet-ompl/pybullet_ompl.py", input]).returncode:
            print("FOUND SOLUTION!")
        else:
            print("DID NOT FIND SOLUTION!")
