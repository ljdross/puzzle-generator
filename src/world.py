from random import seed
from random import random
from random import choice
from subprocess import run
from math import radians
import bpy


class World:
    def __init__(self, config):
        """Initialize all attributes with required world properties."""
        self.name = config["puzzle_name"]
        self.directory = config["dir_for_output"] + "/" + config["puzzle_name"]
        if not config["custom_urdf"]:
            self.number_prismatic_joints = config["number_prismatic_joints"]
            self.number_revolute_joints = config["number_revolute_joints"]
            self.total_number_joints = self.number_prismatic_joints + self.number_revolute_joints
            self.branching_factor = config["branching_factor_target"]
            seed(config["seed_for_randomness"])
            self.allow_clockwise = config["allow_clockwise"]
            self.prismatic_joints_target = self.number_prismatic_joints
            self.revolute_joints_target = self.number_revolute_joints
            self.branching_target = self.branching_factor
            self.start_points = [(0, 0)]
            self.start_point = (0, 0)
        self.goal_state_adjustment = 0.01
        self.start_state = []
        self.goal_state = []
        self.floor_size = config["floor_size"]
        self.export_entity_srdf = config["export_entity_srdf"]
        self.export_mesh_dae = config["export_mesh_dae"]
        self.base_object = None
        self.movable_objects = []

        self.lavender = bpy.data.materials.new("RGB")
        self.lavender.diffuse_color = (0.8, 0.8, 1, 1)
        self.red = bpy.data.materials.new("RGB")
        self.red.diffuse_color = (1, 0, 0, 1)
        self.green = bpy.data.materials.new("RGB")
        self.green.diffuse_color = (0, 1, 0, 1)
        self.white = bpy.data.materials.new("RGB")
        self.white.diffuse_color = (1, 1, 1, 1)

    def reset(self):
        """Delete everything and reset position of 3D cursor."""
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=True)
        bpy.context.scene.cursor.location = (0, 0, 0)
        bpy.context.scene.cursor.rotation_euler = (0, 0, 0)

    def create_cube(self, name, parent=None, location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1), material=None):
        """Create a visual cube object with the appropriate Phobos object properties. Returns cube object."""
        bpy.ops.mesh.primitive_cube_add(location=location, rotation=rotation, scale=tuple(x/2 for x in scale))
        cube = bpy.context.active_object
        cube.active_material = material if material else self.white
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
        bpy.ops.phobos.create_links(location='selected objects', size=8,
        parent_link=True, parent_objects=True, nameformat=name)
        if joint_type:
            bpy.ops.phobos.define_joint_constraints(passive=True, joint_type=joint_type, lower=lower, upper=upper)

    def create_base_link(self):
        """Create a base object to become the base link for all other links.
        If no physical floor is needed, set self.floor_size = 0"""
        self.base_object = self.create_cube(name="visual_cube_base",
        location=(0, 0, -0.1), scale=(self.floor_size, self.floor_size, 0.2), material=self.lavender)
        self.create_link_and_joint(self.base_object, "base_link")

    def create_simple_sliders(self):
        """Create a very simple model that only works with prismatic joints (number_revolute_joints must equal 0)."""
        for i in range(self.number_prismatic_joints):
            if i % 2 == 0:
                self.new_object(location=(i/2, i/-2, 0.1), rotation=(radians(90), 0, 0),
                scale=(0.2, 0.2, 1.6), joint_type='prismatic', upper_limit=1)
            else:
                self.new_object(location=((i-1)/2, ((i-1)/-2)-1, 0.1), rotation=(0, radians(90), 0),
                scale=(0.2, 0.2, 1.6), joint_type='prismatic', upper_limit=1)

    def new_object(self, location, rotation, scale, joint_type, lower_limit=0, upper_limit=0, material=None):
        i = len(self.movable_objects)
        cube = self.create_cube(name="visual_cube" + str(i), parent=self.base_object, location=location,
                                rotation=rotation, scale=scale, material=material)
        self.movable_objects.append(cube)
        self.create_link_and_joint(cube, "link" + str(i), joint_type=joint_type, lower=lower_limit, upper=upper_limit)
        if upper_limit > self.goal_state_adjustment:
            self.goal_state.append(upper_limit - self.goal_state_adjustment)
        elif lower_limit < -self.goal_state_adjustment:
            self.goal_state.append(lower_limit + self.goal_state_adjustment)
        else:
            self.goal_state.append(0)

    def tuple_add(self, a, b):
        return tuple(map(lambda x, y: x + y, a, b))

    def new_prismatic_joint(self):
        # check available positions for prismatic joint
        positions = []
        sp = self.start_points.pop(0)
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
            self.start_points.insert(0, sp)
            return 1

        random_pos = choice(positions)
        self.position_sequence.append(random_pos)
        scale = (1 - self.epsilon, 1 - self.epsilon, 2 - self.epsilon)
        if random_pos == "N":
            # add new prismatic joint at this position
            loc = (sp[0], sp[1] + 0.5, scale[0] / 2)
            rot = (radians(-90), 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (0, 2)))
            self.start_points.append(self.tuple_add(sp, (0, 2)))
        elif random_pos == "E":
            # add new prismatic joint at this position
            loc = (sp[0] + 0.5, sp[1], scale[0] / 2)
            rot = (0, radians(90), 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (1, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (2, 0)))
            self.start_points.append(self.tuple_add(sp, (2, 0)))
        elif random_pos == "S":
            # add new prismatic joint at this position
            loc = (sp[0], sp[1] - 0.5, scale[0] / 2)
            rot = (radians(90), 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (0, -1)))
            self.occupied_fields.append(self.tuple_add(sp, (0, -2)))
            self.start_points.append(self.tuple_add(sp, (0, -2)))
        elif random_pos == "W":
            # add new prismatic joint at this position
            loc = (sp[0] - 0.5, sp[1], scale[0] / 2)
            rot = (0, radians(-90), 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (-1, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (-2, 0)))
            self.start_points.append(self.tuple_add(sp, (-2, 0)))

        # update target counter
        self.prismatic_joints_target -= 1

        return 0

    def new_revolute_joint(self, allow_clockwise=True):
        # check available positions for revolute joint
        positions = []
        sp = self.start_points.pop(0)
        of = self.occupied_fields
        if (self.tuple_add(sp, (0, 1)) not in of and self.tuple_add(sp, (0, 2)) not in of
        and self.tuple_add(sp, (1, 1)) not in of and self.tuple_add(sp, (-1, 1)) not in of):
            # North
            if self.tuple_add(sp, (-1, 2)) not in of and self.tuple_add(sp, (1, 0)) not in of:
                positions.append("N_counterclockwise")
            if self.tuple_add(sp, (1, 2)) not in of and self.tuple_add(sp, (-1, 0)) not in of and allow_clockwise:
                positions.append("N_clockwise")
        if (self.tuple_add(sp, (1, 0)) not in of and self.tuple_add(sp, (2, 0)) not in of
        and self.tuple_add(sp, (1, 1)) not in of and self.tuple_add(sp, (1, -1)) not in of):
            # East
            if self.tuple_add(sp, (2, 1)) not in of and self.tuple_add(sp, (0, -1)) not in of:
                positions.append("E_counterclockwise")
            if self.tuple_add(sp, (0, 1)) not in of and self.tuple_add(sp, (2, -1)) not in of and allow_clockwise:
                positions.append("E_clockwise")
        if (self.tuple_add(sp, (0, -1)) not in of and self.tuple_add(sp, (0, -2)) not in of
        and self.tuple_add(sp, (-1, -1)) not in of and self.tuple_add(sp, (1, -1)) not in of):
            # South
            if self.tuple_add(sp, (-1, 0)) not in of and self.tuple_add(sp, (1, -2)) not in of:
                positions.append("S_counterclockwise")
            if self.tuple_add(sp, (1, 0)) not in of and self.tuple_add(sp, (-1, -2)) not in of and allow_clockwise:
                positions.append("S_clockwise")
        if (self.tuple_add(sp, (-1, 0)) not in of and self.tuple_add(sp, (-2, 0)) not in of
        and self.tuple_add(sp, (-1, 1)) not in of and self.tuple_add(sp, (-1, -1)) not in of):
            # West
            if self.tuple_add(sp, (0, 1)) not in of and self.tuple_add(sp, (-2, -1)) not in of:
                positions.append("W_counterclockwise")
            if self.tuple_add(sp, (-2, 1)) not in of and self.tuple_add(sp, (0, -1)) not in of and allow_clockwise:
                positions.append("W_clockwise")
        if not positions:
            self.start_points.insert(0, sp)
            return 1

        random_pos = choice(positions)
        self.position_sequence.append(random_pos)
        scale = (3 - self.epsilon, 1 - self.epsilon, 1 - self.epsilon)
        if random_pos == "N_counterclockwise":
            # add new revolute joint at this position
            loc = (sp[0], sp[1] + 1, scale[2] / 2)
            rot = (0, 0, radians(90))
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', upper_limit=radians(90))
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (0, 2)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (1, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, 2)))
            self.occupied_fields.append(self.tuple_add(sp, (1, 0)))
            if random() < 0.5:
                self.start_points.append(self.tuple_add(sp, (1, 0)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (-1, 2)))
                    self.branching_target -= 1
            else:
                self.start_points.append(self.tuple_add(sp, (-1, 2)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (1, 0)))
                    self.branching_target -= 1
        elif random_pos == "N_clockwise":
            # add new revolute joint at this position
            loc = (sp[0], sp[1] + 1, scale[2] / 2)
            rot = (0, 0, radians(90))
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', lower_limit=radians(-90))
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (0, 2)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (1, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (1, 2)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, 0)))
            if random() < 0.5:
                self.start_points.append(self.tuple_add(sp, (-1, 0)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (1, 2)))
                    self.branching_target -= 1
            else:
                self.start_points.append(self.tuple_add(sp, (1, 2)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (-1, 0)))
                    self.branching_target -= 1
        elif random_pos == "E_counterclockwise":
            # add new revolute joint at this position
            loc = (sp[0] + 1, sp[1], scale[2] / 2)
            rot = (0, 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', upper_limit=radians(90))
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (1, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (2, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (1, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (1, -1)))
            self.occupied_fields.append(self.tuple_add(sp, (2, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (0, -1)))
            if random() < 0.5:
                self.start_points.append(self.tuple_add(sp, (0, -1)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (2, 1)))
                    self.branching_target -= 1
            else:
                self.start_points.append(self.tuple_add(sp, (2, 1)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (0, -1)))
                    self.branching_target -= 1
        elif random_pos == "E_clockwise":
            # add new revolute joint at this position
            loc = (sp[0] + 1, sp[1], scale[2] / 2)
            rot = (0, 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', lower_limit=radians(-90))
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (1, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (2, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (1, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (1, -1)))
            self.occupied_fields.append(self.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (2, -1)))
            if random() < 0.5:
                self.start_points.append(self.tuple_add(sp, (2, -1)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (0, 1)))
                    self.branching_target -= 1
            else:
                self.start_points.append(self.tuple_add(sp, (0, 1)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (2, -1)))
                    self.branching_target -= 1
        elif random_pos == "S_counterclockwise":
            # add new revolute joint at this position
            loc = (sp[0], sp[1] - 1, scale[2] / 2)
            rot = (0, 0, radians(90))
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', upper_limit=radians(90))
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (0, -1)))
            self.occupied_fields.append(self.tuple_add(sp, (0, -2)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, -1)))
            self.occupied_fields.append(self.tuple_add(sp, (1, -1)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (1, -2)))
            if random() < 0.5:
                self.start_points.append(self.tuple_add(sp, (1, -2)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (-1, 0)))
                    self.branching_target -= 1
            else:
                self.start_points.append(self.tuple_add(sp, (-1, 0)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (1, -2)))
                    self.branching_target -= 1
        elif random_pos == "S_clockwise":
            # add new revolute joint at this position
            loc = (sp[0], sp[1] - 1, scale[2] / 2)
            rot = (0, 0, radians(90))
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', lower_limit=radians(-90))
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (0, -1)))
            self.occupied_fields.append(self.tuple_add(sp, (0, -2)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, -1)))
            self.occupied_fields.append(self.tuple_add(sp, (1, -1)))
            self.occupied_fields.append(self.tuple_add(sp, (1, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, -2)))
            if random() < 0.5:
                self.start_points.append(self.tuple_add(sp, (-1, -2)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (1, 0)))
                    self.branching_target -= 1
            else:
                self.start_points.append(self.tuple_add(sp, (1, 0)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (-1, -2)))
                    self.branching_target -= 1
        elif random_pos == "W_counterclockwise":
            # add new revolute joint at this position
            loc = (sp[0] - 1, sp[1], scale[2] / 2)
            rot = (0, 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', upper_limit=radians(90))
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (-1, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (-2, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, -1)))
            self.occupied_fields.append(self.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (-2, -1)))
            if random() < 0.5:
                self.start_points.append(self.tuple_add(sp, (-2, -1)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (0, 1)))
                    self.branching_target -= 1
            else:
                self.start_points.append(self.tuple_add(sp, (0, 1)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (-2, -1)))
                    self.branching_target -= 1
        elif random_pos == "W_clockwise":
            # add new revolute joint at this position
            loc = (sp[0] - 1, sp[1], scale[2] / 2)
            rot = (0, 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', lower_limit=radians(-90))
            # update occupied fields and start point
            self.occupied_fields.append(self.tuple_add(sp, (-1, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (-2, 0)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (-1, -1)))
            self.occupied_fields.append(self.tuple_add(sp, (-2, 1)))
            self.occupied_fields.append(self.tuple_add(sp, (0, -1)))
            if random() < 0.5:
                self.start_points.append(self.tuple_add(sp, (0, -1)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (-2, 1)))
                    self.branching_target -= 1
            else:
                self.start_points.append(self.tuple_add(sp, (-2, 1)))
                if self.branching_target != 0:
                    self.start_points.append(self.tuple_add(sp, (0, -1)))
                    self.branching_target -= 1

        # update target counter
        self.revolute_joints_target -= 1

        return 0

    def new_joint(self, try_prismatic_first):
        result = 1
        if try_prismatic_first:
            # try creating a prismatic joint
            result = self.new_prismatic_joint()
            if result != 0 and self.revolute_joints_target != 0:
                # if creating a prismatic joint failed and revolute joints are still needed,
                # try creating a revolute joint
                result = self.new_revolute_joint(self.allow_clockwise)

        else:
            # try creating a revolute joint
            result = self.new_revolute_joint(self.allow_clockwise)
            if result != 0 and self.prismatic_joints_target != 0:
                # if creating a revolute joint failed and prismatic joints are still needed,
                # try creating a prismatic joint
                result = self.new_prismatic_joint()

        return result

    def create_gridworld_puzzle(self):
        """Create movable objects to become links for the puzzle (in a grid world)."""
        self.start_points = [(0.5, 0.5)]
        self.occupied_fields = self.start_points.copy()
        self.position_sequence = []
        self.epsilon = 0.1
        try_prismatic: bool

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
                threshold = self.prismatic_joints_target / (self.prismatic_joints_target
                + self.revolute_joints_target)
                if random() < threshold:
                    # create prismatic joint
                    try_prismatic = True
                else:
                    # create revolute joint
                    try_prismatic = False

            # now we know whether we want a prismatic or revolute joint
            # try to create the desired joint
            result = self.new_joint(try_prismatic)
            if result != 0:
                print("ATTEMPT TO CREATE A SEQUENCE FAILED: " + str(self.position_sequence))
                # clean up
                self.goal_state = []
                self.prismatic_joints_target = self.number_prismatic_joints
                self.revolute_joints_target = self.number_revolute_joints
                self.branching_target = self.branching_factor
                self.start_points = [(0, 0)]
                self.movable_objects = []
                return result
        print("SUCCESSFULLY CREATED THE FOLLOWING SEQUENCE: " + str(self.position_sequence))
        return 0

    def remove_last_object(self):
        bpy.ops.object.select_all(action='DESELECT')
        i = str(len(self.movable_objects) - 1)
        bpy.data.objects['collision_cube' + i].select_set(True)
        bpy.data.objects['visual_cube' + i].select_set(True)
        bpy.data.objects['link' + i].select_set(True)
        bpy.ops.object.delete()
        self.movable_objects.pop()
        self.goal_state.pop()

    def set_limit_of_active_object(self, limit, is_prismatic):
        if is_prismatic:
            bpy.context.object.pose.bones["Bone"].constraints["Limit Location"].max_y = limit
        else:
            if limit < 0:
                bpy.context.object.pose.bones["Bone"].constraints["Limit Rotation"].min_x = limit
            else:
                bpy.context.object.pose.bones["Bone"].constraints["Limit Rotation"].max_x = limit
        self.export()
        if limit > self.goal_state_adjustment:
            self.goal_state[-1] = limit - self.goal_state_adjustment
        elif limit < -self.goal_state_adjustment:
            self.goal_state[-1] = limit + self.goal_state_adjustment
        else:
            self.goal_state[-1] = 0

    def sample_joint(self, attempts=50, planning_time=5.):
        self.start_state.append(0)
        first_joint = True if len(self.movable_objects) == 0 else False
        offset = (0, 0)
        threshold = self.prismatic_joints_target / (self.prismatic_joints_target + self.revolute_joints_target)
        is_prismatic: bool
        for i in range(attempts):
            new_point = self.tuple_add(self.start_point, offset)
            offset = (random() * 6 - 3, random() * 6 - 3)
            rot = random() * 360
            if random() < threshold:
                # create immovable (joint limit = 0) prismatic joint
                self.new_object((new_point[0], new_point[1], 0.5), (radians(-90), 0, radians(rot)), (1, 1, 2),
                'prismatic', 0, 0, self.red)
                is_prismatic = True
            else:
                # create immovable (joint limit = 0) revolute joint
                self.new_object((new_point[0], new_point[1], 0.5), (0, 0, radians(rot)), (3, 1, 1),
                'revolute', 0, 0, self.green)
                is_prismatic = False
            self.create_collision(self.movable_objects[-1])
            self.export()
            if first_joint:
                result = 1
            else:
                self.start_state.pop()  # TODO: refactor this
                self.goal_state.pop()
                result = self.test_with_pybullet_ompl(planning_time)
                self.start_state.append(0)
                self.goal_state.append(0)
            if result == 0:
                # can be solved with the immovable joint
                # we do not want that
                # this new joint should block the previous joint
                self.remove_last_object()
                continue
            else:
                # the new (immovable) joint successfully blocks the previously solvable puzzle
                # now make it movable
                if is_prismatic:
                    self.set_limit_of_active_object(random() * 2 + 1, is_prismatic)
                else:
                    limit = random() * 180 - 90
                    if limit > 0:
                        limit += 90
                    else:
                        limit -= 90
                    self.set_limit_of_active_object(radians(limit), is_prismatic)

                # and check solvability again
                if first_joint:
                    result = 0
                else:
                    result = self.test_with_pybullet_ompl(planning_time)
                if result == 0:
                    self.start_point = new_point
                    if is_prismatic:
                        self.prismatic_joints_target -= 1
                    else:
                        self.revolute_joints_target -= 1
                    return 0
                else:
                    self.remove_last_object()

        return 1

    def sample_world(self, attempts=50):
        planning_time = 1
        self.create_collision()
        for i in range(self.total_number_joints):
            result = self.sample_joint(attempts, planning_time)
            if result != 0:
                print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
                print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
                print("could not sample link" + str(i) + " after " + str(attempts) + " attempts")
                print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
                print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
                return result
            planning_time *= 2
        return 0
    
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

    def build_gridworld(self, attempts=50):
        """Build complete model in Blender and export to URDF."""
        self.start_state = [0] * (self.total_number_joints)
        result = 1
        while result != 0:
            self.reset()
            if attempts <= 0:
                return result
            self.create_base_link()
            result = self.create_gridworld_puzzle() # TODO: something better than a discrete grid world
            attempts -= 1

        self.create_collision()
        self.export()
        return 0

    def build_simple_sliders(self):
        """Build complete model in Blender and export to URDF. Create only prismatic joints."""
        self.start_state = [0] * (self.number_prismatic_joints)
        self.reset()
        self.create_base_link()
        self.create_simple_sliders()
        self.create_collision()
        self.export()
        return 0

    def build_sampleworld(self, attempts=50):
        """Build complete model in Blender and export to URDF. Sample random positions for joints."""
        self.reset()
        self.create_base_link()
        result = self.sample_world(attempts)
        if result == 0:
            return 0
        else:
            self.reset()
            return result

    def test_with_pybullet_ompl(self, allowed_planning_time=5., show_gui=False, have_exact_solution=True,
                                planner="RRTConnect"):
        """Test solvability with [pybullet_ompl](https://github.com/lyf44/pybullet_ompl) as a subprocess."""
        input_path = self.directory + "/urdf/" + self.name + ".urdf"
        start_state = str(self.start_state)
        print("self.start_state = " + start_state)
        goal_state = str(self.goal_state)
        print("self.goal_state = " + goal_state)
        result = run(["python3", "pybullet-ompl/pybullet_ompl.py", input_path, start_state, goal_state,
        str(show_gui), str(allowed_planning_time), str(have_exact_solution), planner]).returncode
        if result == 0:
            print("FOUND SOLUTION!")
        else:
            print("DID NOT FIND SOLUTION!")
        return result
