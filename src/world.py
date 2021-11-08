from random import seed
from random import random
from random import choice
import subprocess
from math import radians
import bpy

class World:
    def __init__(self, config):
        """Initialize all attributes with required world properties."""
        self.name = config["puzzle_name"]
        self.directory = config["dir_for_output"] + "/" + config["puzzle_name"]
        self.number_prismatic_joints = config["number_prismatic_joints"]
        self.number_revolute_joints = config["number_revolute_joints"]
        self.total_number_joints = self.number_prismatic_joints + self.number_revolute_joints
        if config["use_floor"]:
            self.floor_size = 16
        else:
            self.floor_size = 0
        self.base_object = None
        self.movable_objects = []
        self.occupied_fields = []

    def reset(self):
        """Delete everything and reset position of 3D cursor."""
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=True)
        bpy.context.scene.cursor.location = (0, 0, 0)
        bpy.context.scene.cursor.rotation_euler = (0, 0, 0)

    def create_cube(self, name, parent=None, location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
        """Create a visual cube object with the appropriate Phobos object properties. Returns cube object."""
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

    def new_object(self, location, rotation, scale, joint_type, lower_limit=0, upper_limit=0):
        i = len(self.movable_objects)
        self.movable_objects.append(self.create_cube(name="visual_cube" + str(i), parent=self.base_object,
        location=location, rotation=rotation, scale=scale))
        self.create_link_and_joint(self.movable_objects[i], "link" + str(i), joint_type=joint_type,
        lower=lower_limit, upper=upper_limit)
    
    def tuple_add(self, a, b):
        return tuple(map(lambda x, y: x + y, a, b))

    def new_prismatic_joint(self):
        # check available positions for prismatic joint
        positions = []
        sp = self.start_point
        of = self.occupied_fields
        if self.tuple_add(sp, (0, 1)) not in of and self.tuple_add(sp, (0, 2)) not in of:
            positions.append("N")
        if self.tuple_add(sp, (1, 0)) not in of and self.tuple_add(sp, (2, 0)) not in of:
            positions.append("E")
        if self.tuple_add(sp, (0, -1)) not in of and self.tuple_add(sp, (0, -2)) not in of:
            positions.append("S")
        if self.tuple_add(sp, (-1, 0)) not in of and self.tuple_add(sp, (-2, 0)) not in of:
            positions.append("W")
        if not positions:
            return 1
        
        seed() # TODO: neccessary?? seems to create better results when seed is reset this way...
        random_pos = choice(positions)
        scale=(0.8, 0.8, 1.8)
        if random_pos == "N":
            # add new prismatic joint at this position
            loc = (sp[0], sp[1] + 0.5, 0.4)
            rot = (radians(-90), 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(sp)
            self.occupied_fields.append(self.tuple_add(sp, (0, 1)))
            self.start_point = self.tuple_add(sp, (0, 2))
        elif random_pos == "E":
            # add new prismatic joint at this position
            loc = (sp[0] + 0.5, sp[1], 0.4)
            rot = (0, radians(90), 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(sp)
            self.occupied_fields.append(self.tuple_add(sp, (1, 0)))
            self.start_point = self.tuple_add(sp, (2, 0))
        elif random_pos == "S":
            # add new prismatic joint at this position
            loc = (sp[0], sp[1] - 0.5, 0.4)
            rot = (radians(90), 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(sp)
            self.occupied_fields.append(self.tuple_add(sp, (0, -1)))
            self.start_point = self.tuple_add(sp, (0, -2))
        elif random_pos == "W":
            # add new prismatic joint at this position
            loc = (sp[0] - 0.5, sp[1], 0.4)
            rot = (0, radians(-90), 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(sp)
            self.occupied_fields.append(self.tuple_add(sp, (-1, 0)))
            self.start_point = self.tuple_add(sp, (-2, 0))

        # update target counter
        self.prismatic_joints_target -= 1

        return 0

    def new_revolute_joint(self):
        # TODO: similar to new_prismatic_joint()
        return 0

    def new_joint(self, prismatic):
        result = 1
        if prismatic:
            # try creating a prismatic joint
            result = self.new_prismatic_joint()
            if result != 0 and self.revolute_joints_target != 0:
                # if creating a prismatic joint failed and revolute joints are still needed,
                # try creating a revolute joint
                result = self.new_revolute_joint()

        else:
            # try creating a revolute joint
            result = self.new_revolute_joint()
            if result != 0 and self.prismatic_joints_target != 0:
                # if creating a revolute joint failed and prismatic joints are still needed,
                # try creating a prismatic joint
                result = self.new_prismatic_joint()

        return result
    
    def create_gridworld_puzzle(self):
        """Create movable objects to become links for the puzzle (in a grid world)."""
        self.prismatic_joints_target = self.number_prismatic_joints
        self.revolute_joints_target = self.number_revolute_joints
        self.start_point = (0, 0)
        try_prismatic = None

        for i in range(self.total_number_joints):
            # in each iteration: try to figure out whether prismatic or revolute joint is needed
            if self.prismatic_joints_target == 0:
                # create revolute joint
                try_prismatic = False
            elif self.revolute_joints_target == 0:
                # create prismatic joint
                try_prismatic = True
            else:
                # create either revolute or prismatic joint (random)
                seed()
                r = random()
                threshold = self.prismatic_joints_target / (self.prismatic_joints_target + self.revolute_joints_target)
                if r < threshold:
                    # create prismatic joint
                    try_prismatic = True
                else:
                    # create revolute joint
                    try_prismatic = False
            
            # now we know whether we want a prismatic or revolute joint
            # try to create the desired joint
            result = self.new_joint(try_prismatic)
            if result != 0:
                return result
        
        return 0
        self.movable_objects.append(self.create_cube(name="visual_cube0", parent=self.base_object,
        location=(0, 0.5, 0.4), rotation=(radians(-90), 0, 0), scale=(0.8, 0.8, 1.8)))
        self.create_link_and_joint(self.movable_objects[0], "link0", joint_type='prismatic', upper=1)

        self.movable_objects.append(self.create_cube(name="visual_cube1", parent=self.base_object,
        location=(-0.5, 2, 0.9), rotation=(0, 0, 0), scale=(1.8, 0.1, 1.8)))
        self.create_link_and_joint(self.movable_objects[1], "link1", joint_type='revolute', upper=radians(90))

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
        # self.create_simple_sliders()
        self.create_gridworld_puzzle() # TODO: something better than a discrete grid world, works only with prismatic joints so far
        self.create_collision()
        self.export()

    def test_with_pybullet_ompl(self, show_gui=True, allowed_planning_time=5.):
        """Test solvability with [pybullet_ompl](https://github.com/lyf44/pybullet_ompl) as a subprocess."""
        input_path = self.directory + "/urdf/" + self.name + ".urdf"
        start_state = str([0] * (self.total_number_joints))
        goal_state = str([1] * (self.total_number_joints))
        # goal_state = str([1, radians(90)])
        result = subprocess.run(["python3", "pybullet-ompl/pybullet_ompl.py", input_path, start_state, goal_state,
        str(show_gui), str(allowed_planning_time)]).returncode
        if result == 0:
            print("FOUND SOLUTION!")
        else:
            print("DID NOT FIND SOLUTION!")
