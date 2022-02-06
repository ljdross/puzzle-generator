from random import seed, random, choice
import bpy
import os
import sys
DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
import calc
import color


class World:
    def __init__(self, config):
        """Initialize all attributes with required world properties."""
        self.name = config["puzzle_name"]
        self.directory = config["dir_for_output"] + "/" + config["puzzle_name"]
        self.urdf_path = self.directory + "/urdf/" + self.name + ".urdf"
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
            self.upper_limit_prismatic = (2, 4)  # interval for the random upper limit, the lower limit is always 0
            self.upper_limit_revolute = (calc.RAD90, calc.RAD180)
            self.prismatic_length = 2
            self.revolute_length = 3
        self.goal_adjustment = 0.0001
        self.start_state = []
        self.goal_space = []
        self.floor_size = config["floor_size"]
        self.export_entity_srdf = config["export_entity_srdf"]
        self.export_mesh_dae = config["export_mesh_dae"]
        self.base_object = None
        self.movable_objects = []

    def reset(self):
        """Delete everything and reset position of 3D cursor."""
        self.movable_objects = []
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=True)
        bpy.context.scene.cursor.location = (0, 0, 0)
        bpy.context.scene.cursor.rotation_euler = (0, 0, 0)

    def create_cube(self, name, parent=None, location=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1), material=None):
        """Create a visual cube object with the appropriate Phobos object properties. Returns cube object."""
        bpy.ops.mesh.primitive_cube_add(location=location, rotation=rotation, scale=tuple(x/2 for x in scale))
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
        bpy.ops.phobos.create_links(location='selected objects', size=8,
                                    parent_link=True, parent_objects=True, nameformat=name)
        if joint_type:
            bpy.ops.phobos.define_joint_constraints(passive=True, joint_type=joint_type, lower=lower, upper=upper)

    def create_base_link(self):
        """Create a base object to become the base link for all other links.
        If no physical floor is needed, set self.floor_size = 0"""
        self.base_object = self.create_cube(name="visual_cube_base",
                                            location=(0, 0, -0.1), scale=(self.floor_size, self.floor_size, 0.2),
                                            material=color.LAVENDER)
        self.create_link_and_joint(self.base_object, "base_link")

    def create_simple_sliders_puzzle(self):
        """Create a very simple model that only works with prismatic joints (number_revolute_joints must equal 0)."""
        for i in range(self.number_prismatic_joints):
            if i % 2 == 0:
                self.new_object(location=(i / 2, i / -2, 0.1), rotation=(calc.RAD90, 0, 0),
                                scale=(0.2, 0.2, 1.6), joint_type='prismatic', upper_limit=1)
            else:
                self.new_object(location=((i - 1) / 2, ((i - 1) / -2) - 1, 0.1), rotation=(0, calc.RAD90, 0),
                                scale=(0.2, 0.2, 1.6), joint_type='prismatic', upper_limit=1)

    def new_object(self, location, rotation, scale, joint_type, lower_limit=0, upper_limit=0, material=None,
                   add_to_goal_space=True):
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
        if add_to_goal_space:
            if abs(lower_limit - upper_limit) > 2 * self.goal_adjustment:
                if lower_limit:
                    lower_limit += self.goal_adjustment
                if upper_limit:
                    upper_limit -= self.goal_adjustment
            self.goal_space.append((lower_limit, upper_limit))

    def new_prismatic_joint(self):
        # check available positions for prismatic joint
        positions = []
        sp = self.start_points.pop(0)
        of = self.occupied_fields
        if calc.tuple_add(sp, (0, 1)) not in of and calc.tuple_add(sp, (0, 2)) not in of:
            positions.append("N")
        if calc.tuple_add(sp, (1, 0)) not in of and calc.tuple_add(sp, (2, 0)) not in of:
            positions.append("E")
        if calc.tuple_add(sp, (0, -1)) not in of and calc.tuple_add(sp, (0, -2)) not in of:
            positions.append("S")
        if calc.tuple_add(sp, (-1, 0)) not in of and calc.tuple_add(sp, (-2, 0)) not in of:
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
            rot = (-calc.RAD90, 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, 2)))
            self.start_points.append(calc.tuple_add(sp, (0, 2)))
        elif random_pos == "E":
            # add new prismatic joint at this position
            loc = (sp[0] + 0.5, sp[1], scale[0] / 2)
            rot = (0, calc.RAD90, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (2, 0)))
            self.start_points.append(calc.tuple_add(sp, (2, 0)))
        elif random_pos == "S":
            # add new prismatic joint at this position
            loc = (sp[0], sp[1] - 0.5, scale[0] / 2)
            rot = (calc.RAD90, 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (0, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, -2)))
            self.start_points.append(calc.tuple_add(sp, (0, -2)))
        elif random_pos == "W":
            # add new prismatic joint at this position
            loc = (sp[0] - 0.5, sp[1], scale[0] / 2)
            rot = (0, -calc.RAD90, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (-2, 0)))
            self.start_points.append(calc.tuple_add(sp, (-2, 0)))

        # update target counter
        self.prismatic_joints_target -= 1

        return 0

    def new_revolute_joint(self, allow_clockwise=True):
        # check available positions for revolute joint
        positions = []
        sp = self.start_points.pop(0)
        of = self.occupied_fields
        if (calc.tuple_add(sp, (0, 1)) not in of and calc.tuple_add(sp, (0, 2)) not in of
                and calc.tuple_add(sp, (1, 1)) not in of and calc.tuple_add(sp, (-1, 1)) not in of):
            # North
            if calc.tuple_add(sp, (-1, 2)) not in of and calc.tuple_add(sp, (1, 0)) not in of:
                positions.append("N_counterclockwise")
            if calc.tuple_add(sp, (1, 2)) not in of and calc.tuple_add(sp, (-1, 0)) not in of and allow_clockwise:
                positions.append("N_clockwise")
        if (calc.tuple_add(sp, (1, 0)) not in of and calc.tuple_add(sp, (2, 0)) not in of
                and calc.tuple_add(sp, (1, 1)) not in of and calc.tuple_add(sp, (1, -1)) not in of):
            # East
            if calc.tuple_add(sp, (2, 1)) not in of and calc.tuple_add(sp, (0, -1)) not in of:
                positions.append("E_counterclockwise")
            if calc.tuple_add(sp, (0, 1)) not in of and calc.tuple_add(sp, (2, -1)) not in of and allow_clockwise:
                positions.append("E_clockwise")
        if (calc.tuple_add(sp, (0, -1)) not in of and calc.tuple_add(sp, (0, -2)) not in of
                and calc.tuple_add(sp, (-1, -1)) not in of and calc.tuple_add(sp, (1, -1)) not in of):
            # South
            if calc.tuple_add(sp, (-1, 0)) not in of and calc.tuple_add(sp, (1, -2)) not in of:
                positions.append("S_counterclockwise")
            if calc.tuple_add(sp, (1, 0)) not in of and calc.tuple_add(sp, (-1, -2)) not in of and allow_clockwise:
                positions.append("S_clockwise")
        if (calc.tuple_add(sp, (-1, 0)) not in of and calc.tuple_add(sp, (-2, 0)) not in of
                and calc.tuple_add(sp, (-1, 1)) not in of and calc.tuple_add(sp, (-1, -1)) not in of):
            # West
            if calc.tuple_add(sp, (0, 1)) not in of and calc.tuple_add(sp, (-2, -1)) not in of:
                positions.append("W_counterclockwise")
            if calc.tuple_add(sp, (-2, 1)) not in of and calc.tuple_add(sp, (0, -1)) not in of and allow_clockwise:
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
            rot = (0, 0, calc.RAD90)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', upper_limit=calc.RAD90)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, 2)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 2)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, 0)))
            if random() < 0.5:
                self.start_points.append(calc.tuple_add(sp, (1, 0)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (-1, 2)))
                    self.branching_target -= 1
            else:
                self.start_points.append(calc.tuple_add(sp, (-1, 2)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (1, 0)))
                    self.branching_target -= 1
        elif random_pos == "N_clockwise":
            # add new revolute joint at this position
            loc = (sp[0], sp[1] + 1, scale[2] / 2)
            rot = (0, 0, calc.RAD90)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', lower_limit=-calc.RAD90)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, 2)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, 2)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 0)))
            if random() < 0.5:
                self.start_points.append(calc.tuple_add(sp, (-1, 0)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (1, 2)))
                    self.branching_target -= 1
            else:
                self.start_points.append(calc.tuple_add(sp, (1, 2)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (-1, 0)))
                    self.branching_target -= 1
        elif random_pos == "E_counterclockwise":
            # add new revolute joint at this position
            loc = (sp[0] + 1, sp[1], scale[2] / 2)
            rot = (0, 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', upper_limit=calc.RAD90)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (2, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (2, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, -1)))
            if random() < 0.5:
                self.start_points.append(calc.tuple_add(sp, (0, -1)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (2, 1)))
                    self.branching_target -= 1
            else:
                self.start_points.append(calc.tuple_add(sp, (2, 1)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (0, -1)))
                    self.branching_target -= 1
        elif random_pos == "E_clockwise":
            # add new revolute joint at this position
            loc = (sp[0] + 1, sp[1], scale[2] / 2)
            rot = (0, 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', lower_limit=-calc.RAD90)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (2, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (2, -1)))
            if random() < 0.5:
                self.start_points.append(calc.tuple_add(sp, (2, -1)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (0, 1)))
                    self.branching_target -= 1
            else:
                self.start_points.append(calc.tuple_add(sp, (0, 1)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (2, -1)))
                    self.branching_target -= 1
        elif random_pos == "S_counterclockwise":
            # add new revolute joint at this position
            loc = (sp[0], sp[1] - 1, scale[2] / 2)
            rot = (0, 0, calc.RAD90)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', upper_limit=calc.RAD90)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (0, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, -2)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, -2)))
            if random() < 0.5:
                self.start_points.append(calc.tuple_add(sp, (1, -2)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (-1, 0)))
                    self.branching_target -= 1
            else:
                self.start_points.append(calc.tuple_add(sp, (-1, 0)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (1, -2)))
                    self.branching_target -= 1
        elif random_pos == "S_clockwise":
            # add new revolute joint at this position
            loc = (sp[0], sp[1] - 1, scale[2] / 2)
            rot = (0, 0, calc.RAD90)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', lower_limit=-calc.RAD90)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (0, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, -2)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, -2)))
            if random() < 0.5:
                self.start_points.append(calc.tuple_add(sp, (-1, -2)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (1, 0)))
                    self.branching_target -= 1
            else:
                self.start_points.append(calc.tuple_add(sp, (1, 0)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (-1, -2)))
                    self.branching_target -= 1
        elif random_pos == "W_counterclockwise":
            # add new revolute joint at this position
            loc = (sp[0] - 1, sp[1], scale[2] / 2)
            rot = (0, 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', upper_limit=calc.RAD90)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (-2, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (-2, -1)))
            if random() < 0.5:
                self.start_points.append(calc.tuple_add(sp, (-2, -1)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (0, 1)))
                    self.branching_target -= 1
            else:
                self.start_points.append(calc.tuple_add(sp, (0, 1)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (-2, -1)))
                    self.branching_target -= 1
        elif random_pos == "W_clockwise":
            # add new revolute joint at this position
            loc = (sp[0] - 1, sp[1], scale[2] / 2)
            rot = (0, 0, 0)
            self.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute', lower_limit=-calc.RAD90)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (-2, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (-1, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (-2, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, -1)))
            if random() < 0.5:
                self.start_points.append(calc.tuple_add(sp, (0, -1)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (-2, 1)))
                    self.branching_target -= 1
            else:
                self.start_points.append(calc.tuple_add(sp, (-2, 1)))
                if self.branching_target != 0:
                    self.start_points.append(calc.tuple_add(sp, (0, -1)))
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
                self.goal_space = []
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

    def set_limit_of_active_object_and_add_to_goal_space(self, limit, is_prismatic):
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

        # TODO: remove the following and change name of this function
        if limit > self.goal_adjustment:
            self.goal_space.append((0, limit - self.goal_adjustment))
        elif limit < -self.goal_adjustment:
            self.goal_space.append((limit + self.goal_adjustment, 0))
        else:
            self.goal_space.append((0, 0))

    def get_random_limit(self, is_prismatic):
        """
        Return a random limit within the interval upper_limit_prismatic or upper_limit_revolute respectively.
        If not prismatic the return value will be in radians and there is a 50 % chance that it will be negative.
        """
        if is_prismatic:
            diff = self.upper_limit_prismatic[1] - self.upper_limit_prismatic[0]
            return random() * diff + self.upper_limit_prismatic[0]
        else:
            diff = self.upper_limit_revolute[1] - self.upper_limit_revolute[0]
            limit = random() * diff * 2 - diff
            if limit > 0:
                limit += self.upper_limit_revolute[0]
            else:
                limit -= self.upper_limit_revolute[0]
            return limit

    def calculate_next_start_point(self, is_prismatic, pos, rotation, limit):
        """
        Calculate the center of the sampling area for the next joint to maximize the chance of blocking the previous
        joint.
        """
        if is_prismatic:
            link_oriented = calc.rotate((0, self.prismatic_length / 2 + limit / 2), rotation)
        else:
            link_oriented = calc.rotate((self.revolute_length, 0), rotation + calc.RAD90)
        return calc.tuple_add(pos, link_oriented)

    def sample_joint(self, attempts, planning_time, first_test_advantage=1.5, area_size=3):
        """
        Assume that the first link+joint as been placed already.
        Try to place a new link+joint at a random position within in a continuous interval so that the puzzle
        1. is UNsolvable if the new link+joint can NOT be moved
        2. is SOLVABLE if the new link+joint CAN be moved
        """
        threshold = self.prismatic_joints_target / (self.prismatic_joints_target + self.revolute_joints_target)
        is_prismatic: bool
        for i in range(attempts):
            offset = (random() * area_size - area_size / 2, random() * area_size - area_size / 2)
            new_point = calc.tuple_add(self.start_point, offset)
            rotation = random() * calc.RAD360
            if random() < threshold:
                # create immovable prismatic joint (joint limits = 0)
                self.new_object((new_point[0], new_point[1], 0.5), (-calc.RAD90, 0, rotation), (1, 1, self.prismatic_length),
                                'prismatic', lower_limit=0, upper_limit=0, add_to_goal_space=False)
                is_prismatic = True
            else:
                # create immovable revolute joint (joint limits = 0)
                self.new_object((new_point[0], new_point[1], 0.5), (0, 0, rotation), (self.revolute_length, 1, 1),
                                'revolute', lower_limit=0, upper_limit=0, add_to_goal_space=False)
                is_prismatic = False
            self.create_collision(self.movable_objects[-1])
            self.export()
            result = self.test_with_pybullet_ompl(planning_time * first_test_advantage)
            if result == 0:
                # can be solved with the immovable joint
                # we do not want that
                # this new joint should block the previous joint
                self.remove_last_object()
                continue
            else:
                # the new (immovable) joint successfully blocks the previously solvable puzzle
                # now make it movable
                limit = self.get_random_limit(is_prismatic)
                self.set_limit_of_active_object_and_add_to_goal_space(limit, is_prismatic)
                self.start_state.append(0)

                # and check solvability again
                result = self.test_with_pybullet_ompl(planning_time)
                if result == 0:
                    self.start_point = self.calculate_next_start_point(is_prismatic, new_point, rotation, limit)
                    if is_prismatic:
                        self.prismatic_joints_target -= 1
                    else:
                        self.revolute_joints_target -= 1
                    return 0
                else:
                    self.start_state.pop()
                    self.goal_space.pop()
                    self.remove_last_object()

        return 1

    def sample_first_joint(self):
        """
        Place the first link+joint at start_point
        """
        self.start_state.append(0)
        threshold = self.prismatic_joints_target / (self.prismatic_joints_target + self.revolute_joints_target)
        rotation = random() * calc.RAD360
        # since this is the first joint, we do not need to check solvability
        # but the goal is to move link0 to a specific location
        # so this dimension in the goal space must be narrowed
        if random() < threshold:
            # create prismatic joint
            limit = self.get_random_limit(True)
            self.goal_space.append((limit - self.goal_adjustment, limit - self.goal_adjustment))
            self.new_object((self.start_point[0], self.start_point[1], 0.5), (-calc.RAD90, 0, rotation), (1, 1, self.prismatic_length),
                            'prismatic', lower_limit=0, upper_limit=limit, add_to_goal_space=False)
            self.prismatic_joints_target -= 1
            self.start_point = self.calculate_next_start_point(True, self.start_point, rotation, limit)
        else:
            # create revolute joint
            limit = self.get_random_limit(False)
            if limit > 0:
                self.goal_space.append((limit - self.goal_adjustment, limit - self.goal_adjustment))
                self.new_object((self.start_point[0], self.start_point[1], 0.5), (0, 0, rotation), (self.revolute_length, 1, 1),
                                'revolute', lower_limit=0, upper_limit=limit, add_to_goal_space=False)
            else:
                self.goal_space.append((limit + self.goal_adjustment, limit + self.goal_adjustment))
                self.new_object((self.start_point[0], self.start_point[1], 0.5), (0, 0, rotation), (self.revolute_length, 1, 1),
                                'revolute', lower_limit=limit, upper_limit=0, add_to_goal_space=False)
            self.revolute_joints_target -= 1
            self.start_point = self.calculate_next_start_point(False, self.start_point, rotation, limit)

    def create_sampleworld_puzzle(self, attempts=50, planning_time=0.1, planning_time_multiplier=2):
        """
        Sample links with joints interatively
        """
        self.sample_first_joint()
        self.create_collision()
        for i in range(1, self.total_number_joints):
            result = self.sample_joint(attempts, planning_time)
            if result != 0:
                print("\U000026D4 " * 64)
                print("Could NOT sample link" + str(i), "after", attempts, "attempts!")
                print("\U000026D4 " * 64)
                return result
            print("Successfully sampled link" + str(i), "\U000026F3 " * i)
            planning_time *= planning_time_multiplier
        print("\U000026F3 " * 32)
        print("SUCCESS! Sampled", self.total_number_joints, "links!")
        print("\U000026F3 " * 32)
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
        self.start_state = [0] * self.total_number_joints
        result = 1
        while result != 0:
            self.reset()
            if attempts <= 0:
                return result
            self.create_base_link()
            result = self.create_gridworld_puzzle()
            attempts -= 1

        self.create_collision()
        self.export()
        return 0

    def build_simple_sliders_world(self):
        """Build complete model in Blender and export to URDF. Create only prismatic joints."""
        self.start_state = [0] * self.number_prismatic_joints
        self.reset()
        self.create_base_link()
        self.create_simple_sliders_puzzle()
        self.create_collision()
        self.export()
        return 0

    def build_sampleworld(self, attempts=50):
        """Build complete model in Blender and export to URDF. Sample random positions for joints."""
        self.reset()
        self.create_base_link()
        result = self.create_sampleworld_puzzle(attempts)
        if result == 0:
            return 0
        else:
            self.reset()
            return result
