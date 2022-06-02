from random import seed, random, choice, shuffle
import os
import sys

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from world import BlenderWorld
from pybullet_simulation import solve
import calc
import color


class PuzzleSampler:
    def __init__(self, config, world: BlenderWorld):
        self.world = world
        self.floor_size = config["floor_size"]
        self.scaling = config["scaling"]
        self.number_prismatic_joints = config["number_prismatic_joints"]
        self.number_revolute_joints = config["number_revolute_joints"]
        self.total_number_joints = self.number_prismatic_joints + self.number_revolute_joints
        self.attempts = config["attempts"]
        seed(config["seed_for_randomness"])
        self.create_handle = config["create_handle"]
        self.prismatic_joints_target = self.number_prismatic_joints
        self.revolute_joints_target = self.number_revolute_joints
        self.start_point = (0, 0)
        self.prismatic_length = 2
        self.revolute_length = 3
        self.goal_adjustment = 0.  # e.g. 0.0001
        self.start_state = []
        self.goal_space = []

    def build(self):
        raise NotImplementedError

    def goal_space_append_with_adjustment(self, limits: tuple):
        if len(limits) != 2:
            print("goal_space_append_with_adjustment(): INPUT TUPLE LENGTH IS NOT 2")
        lower_limit = limits[0]
        upper_limit = limits[1]
        if type(lower_limit) == int and type(upper_limit) == int:
            self.goal_space.append((lower_limit, upper_limit))
            return
        if abs(upper_limit - lower_limit) > 2 * self.goal_adjustment:
            if type(lower_limit) != int:
                lower_limit += self.goal_adjustment
            if type(upper_limit) != int:
                upper_limit -= self.goal_adjustment
        else:
            # if lower and upper limit are close to each other (e.g. the same), move both of them closer to 0
            if lower_limit > self.goal_adjustment:
                lower_limit -= self.goal_adjustment
            elif lower_limit < -self.goal_adjustment:
                lower_limit += self.goal_adjustment
            else:
                lower_limit = 0
            if upper_limit > self.goal_adjustment:
                upper_limit -= self.goal_adjustment
            elif upper_limit < -self.goal_adjustment:
                upper_limit += self.goal_adjustment
            else:
                upper_limit = 0

        self.goal_space.append((lower_limit, upper_limit))

    def goal_space_narrow(self, dimension: int):
        if len(self.goal_space) > dimension:
            lower = self.goal_space[dimension][0]
            upper = self.goal_space[dimension][1]
            if abs(lower) > abs(upper):
                self.goal_space[dimension] = (lower, lower)
            else:
                self.goal_space[dimension] = (upper, upper)
        else:
            print("goal_space_narrow(dimension): goal_space has only", len(self.goal_space), "dimensions. dimension",
                  dimension, "is not available (starting indexing of dimensions with 0!)")

    def get_limits_tuple(self, limit):
        if limit < 0:
            return limit, 0
        else:
            return 0, limit


class SimpleSlidersSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        if "puzzle_name" in config:
            world.update_name(config["puzzle_name"])
        else:
            world.update_name("simple_sliders")
        self.gap = config["gap"]

    def _create_simple_sliders_puzzle(self):
        """
        Create a very simple model that only works with prismatic joints
        (ignore number_revolute_joints).
        """
        scale = (1.8 - 2 * self.gap, 0.2, 0.2)
        for i in range(self.number_prismatic_joints):
            if i % 2 == 0:
                location = (i / 2, i / -2, 0.1)
                rotation = (0, 0, -calc.RAD90)
            else:
                location = ((i - 1) / 2, ((i - 1) / -2) - 1, 0.1)
                rotation = (0, 0, 0)
            self.world.new_link(location, rotation, scale, 'prismatic', upper_limit=1, create_handle=self.create_handle,
                                joint_axis=(1, 0, 0))
            self.goal_space.append((0, 1))
        self.goal_space_narrow(dimension=0)
        self.start_state = [0] * self.number_prismatic_joints
        self.world.create_goal_duplicate((1, 0, 0))

    def build(self):
        """Build complete model in Blender and export to URDF. Create only prismatic joints."""
        self.world.initialize(self.floor_size)
        self._create_simple_sliders_puzzle()
        self.world.export()
        self.world.render_images()
        return 0


class GridWorldSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        if "puzzle_name" in config:
            world.update_name(config["puzzle_name"])
        else:
            world.update_name("grid_world")
        self.allow_clockwise = config["allow_clockwise"]
        self.epsilon = config["epsilon"]
        self.branching_per_revolute_joint = config["branching_per_revolute_joint"]
        self.branching_factor = config["branching_factor_target"]
        self.branching_target = self.branching_factor
        self.start_points = [(0.5, 0.5)]
        self.occupied_fields = self.start_points.copy()
        self.position_sequence = []
        self.blender_operations_queue = []
        self.fields_to_occupy = {
            # prismatic
            # last field is potential new start point
            "N": ((0, 1), (0, 2)),
            "E": ((1, 0), (2, 0)),
            "S": ((0, -1), (0, -2)),
            "W": ((-1, 0), (-2, 0)),

            # revolute
            # last two fields are potential new start points
            "N_counterclockwise":   ((0, 1), (0, 2), (1, 1), (-1, 1), (-1, 2), (1, 0)),
            "N_clockwise":          ((0, 1), (0, 2), (1, 1), (-1, 1), (1, 2), (-1, 0)),
            "E_counterclockwise":   ((1, 0), (2, 0), (1, -1), (1, 1), (2, 1), (0, -1)),
            "E_clockwise":          ((1, 0), (2, 0), (1, -1), (1, 1), (0, 1), (2, -1)),
            "S_counterclockwise":   ((0, -1), (0, -2), (-1, -1), (1, -1), (-1, 0), (1, -2)),
            "S_clockwise":          ((0, -1), (0, -2), (-1, -1), (1, -1), (1, 0), (-1, -2)),
            "W_counterclockwise":   ((-1, 0), (-2, 0), (-1, 1), (-1, -1), (0, 1), (-2, -1)),
            "W_clockwise":          ((-1, 0), (-2, 0), (-1, 1), (-1, -1), (-2, 1), (0, -1)),
        }
        self.link_position = {
            # ((x_location, y_location), rotation, limit)

            # prismatic
            "N": ((0, 0.5), calc.RAD90, 1),
            "E": ((0.5, 0), 0, 1),
            "S": ((0, -0.5), -calc.RAD90, 1),
            "W": ((-0.5, 0), calc.RAD180, 1),

            # revolute
            "N_counterclockwise":   ((0, 1), calc.RAD90, calc.RAD90),
            "N_clockwise":          ((0, 1), calc.RAD90, -calc.RAD90),
            "E_counterclockwise":   ((1, 0), 0, calc.RAD90),
            "E_clockwise":          ((1, 0), 0, -calc.RAD90),
            "S_counterclockwise":   ((0, -1), calc.RAD90, calc.RAD90),
            "S_clockwise":          ((0, -1), calc.RAD90, -calc.RAD90),
            "W_counterclockwise":   ((-1, 0), 0, calc.RAD90),
            "W_clockwise":          ((-1, 0), 0, -calc.RAD90),
        }

    def available(self, direction):
        """
        Return True if all fields in the given direction are available.
        Assume self.start_points[0] as current position!
        """
        fields = self.fields_to_occupy[direction]
        for field in fields:
            if calc.tuple_add(self.start_points[0], field) in self.occupied_fields:
                return False
        return True

    def occupy(self, direction):
        """
        Extend self.occupied_fields by newly occupied fields in the given direction.
        Assume self.start_points[0] as current position!
        """
        fields = self.fields_to_occupy[direction]
        for field in fields:
            self.occupied_fields.append(calc.tuple_add(self.start_points[0], field))

    def _add_new_start_points(self, direction):
        if len(direction) == 1:  # prismatic
            direction_field = self.fields_to_occupy[direction][-1]
            new_start_point = calc.tuple_add(self.start_points[0], direction_field)
            self.start_points.append(new_start_point)
        else:  # revolute
            new_start_points_amount = 1 + self.branching_per_revolute_joint
            direction_fields = self.fields_to_occupy[direction][-new_start_points_amount:]
            new_start_points = [calc.tuple_add(self.start_points[0], df) for df in direction_fields]
            shuffle(new_start_points)
            self.start_points.append(new_start_points.pop())
            for new_start_point in new_start_points:
                if self.branching_target != 0:
                    self.start_points.append(new_start_point)
                    self.branching_target -= 1
                else:
                    break

    def _place_link(self, direction, prismatic: bool):
        if prismatic:
            scale = (2 - self.epsilon, 1 - self.epsilon, 1 - self.epsilon)
        else:
            scale = (3 - self.epsilon, 1 - self.epsilon, 1 - self.epsilon)
        loc, rot, limit = self.link_position[direction]
        sp = self.start_points[0]
        loc = (sp[0] + loc[0], sp[1] + loc[1], scale[2] / 2)
        rot = (0, 0, rot)

        loc = calc.tuple_scale(loc, self.scaling)
        scale = calc.tuple_scale(scale, self.scaling)
        if prismatic:
            limit = round(limit * self.scaling, 5)

            call = lambda: self.world.new_link(loc, rot, scale, 'prismatic', upper_limit=limit,
                                               create_handle=self.create_handle, joint_axis=(1, 0, 0))
            self.prismatic_joints_target -= 1
        else:
            call = lambda: self.world.new_link(loc, rot, scale, 'revolute', auto_limit=limit,
                                               create_handle=self.create_handle, hinge_diameter=None)
            self.revolute_joints_target -= 1
        self.blender_operations_queue.append(call)

        self.goal_space_append_with_adjustment(self.get_limits_tuple(limit))

    def _choose_link(self, prismatic: bool):
        if prismatic:
            if self.prismatic_joints_target == 0:
                return 1

            positions = ("N", "E", "S", "W")

        else:
            if self.revolute_joints_target == 0:
                return 1

            if self.allow_clockwise:
                positions = ("N_counterclockwise", "N_clockwise",
                             "E_counterclockwise", "E_clockwise",
                             "S_counterclockwise", "S_clockwise",
                             "W_counterclockwise", "W_clockwise")
            else:
                positions = ("N_counterclockwise",
                             "E_counterclockwise",
                             "S_counterclockwise",
                             "W_counterclockwise")

        available_positions = tuple(p for p in positions if self.available(p))
        if not available_positions:
            return 1

        random_pos = choice(available_positions)
        self.position_sequence.append(random_pos)
        self.occupy(random_pos)
        self._add_new_start_points(random_pos)
        self._place_link(random_pos, prismatic)
        self.start_points.pop(0)

        return 0

    def _new_joint(self, try_prismatic_first):
        result = 1
        # try creating either a prismatic or a revolute joint
        result = self._choose_link(prismatic=try_prismatic_first)
        if result != 0:
            # if creating that type of joint failed, try creating the other type of joint
            result = self._choose_link(prismatic=not try_prismatic_first)

        return result

    def _clean_up(self):
        self.goal_space = []
        self.prismatic_joints_target = self.number_prismatic_joints
        self.revolute_joints_target = self.number_revolute_joints
        self.branching_target = self.branching_factor
        self.start_points = [(0.5, 0.5)]
        self.occupied_fields = self.start_points.copy()
        self.position_sequence = []
        self.blender_operations_queue = []

    def _create_grid_world_puzzle(self):
        """Create movable objects to become links for the puzzle (in a grid world)."""
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
                threshold = self.prismatic_joints_target / (self.prismatic_joints_target + self.revolute_joints_target)
                if random() < threshold:
                    # create prismatic joint
                    try_prismatic = True
                else:
                    # create revolute joint
                    try_prismatic = False

            # now we know whether we want a prismatic or revolute joint
            # try to create the desired joint
            result = self._new_joint(try_prismatic)
            if result != 0:
                print("ATTEMPT TO CREATE A SEQUENCE FAILED: " + str(self.position_sequence))
                self._clean_up()
                return result
        print("SUCCESSFULLY CREATED THE FOLLOWING SEQUENCE: " + str(self.position_sequence))
        for operation in self.blender_operations_queue:
            operation()
        self.goal_space_narrow(dimension=0)
        goal_limit = self.goal_space[0][1]
        if len(self.position_sequence[0]) == 1:  # first joint is prismatic
            self.world.create_goal_duplicate((goal_limit, 0, 0))
        else:  # revolute
            self.world.create_goal_duplicate(rotation_offset=(0, 0, goal_limit))

        return 0

    def build(self):
        """Build complete model in Blender and export to URDF."""
        self.start_state = [0] * self.total_number_joints
        for _ in range(self.attempts):
            self.world.initialize(self.floor_size)
            result = self._create_grid_world_puzzle()
            if result == 0:
                self.world.export()
                self.world.render_images()
                return 0
        print("GridWorldSampler failed!")
        return result


class ContinuousSpaceSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        if "puzzle_name" in config:
            world.update_name(config["puzzle_name"])
        else:
            world.update_name("continuous_space")
        self.planning_time = config["start_planning_time"]
        self.initial_planning_time = self.planning_time
        self.planning_time_multiplier = config["planning_time_multiplier"]
        self.first_test_time_multiplier = config["first_test_time_multiplier"]
        self.area_size = config["area_size"]
        self.upper_limit_prismatic = config["upper_limit_prismatic"]
        self.upper_limit_revolute = config["upper_limit_revolute"]
        self.attempts_per_link = config["attempts_per_link"]
        self.start_points = [(0, 0)]

    def _get_random_limit(self, is_prismatic):
        """
        Return a random limit within the interval upper_limit_prismatic or upper_limit_revolute respectively.
        If not prismatic the return value will be in radians and there is a 50 % chance that it will be negative.
        """
        if is_prismatic:
            diff = self.upper_limit_prismatic[1] - self.upper_limit_prismatic[0]
            return round(random() * diff + self.upper_limit_prismatic[0], 5)
        else:
            diff = self.upper_limit_revolute[1] - self.upper_limit_revolute[0]
            limit = random() * diff * 2 - diff
            if limit > 0:
                limit += self.upper_limit_revolute[0]
            else:
                limit -= self.upper_limit_revolute[0]
            return round(limit, 5)

    def _calculate_next_start_point(self, is_prismatic, pos, rotation, limit):
        """
        Remove the previous start point and append new start point(s).
        Calculate the center of the sampling area for the next joint to maximize the chance of blocking the previous
        joint.
        """
        self.start_points.pop(0)
        if is_prismatic:
            link_oriented = calc.rotate((0, self.prismatic_length / 2 + limit / 2), rotation)
            self.start_points.append(calc.tuple_add(pos, link_oriented))
        else:
            link_oriented = calc.rotate((self.revolute_length, 0), rotation - calc.RAD90)
            self.start_points.append(calc.tuple_add(pos, link_oriented))
            link_oriented = calc.rotate((self.revolute_length, 0), rotation + calc.RAD90)
            self.start_points.append(calc.tuple_add(pos, link_oriented))

    def _sample_first_joint(self):
        """
        Place the first link+joint at start_point
        """
        self.start_state.append(0)
        threshold = self.prismatic_joints_target / (self.prismatic_joints_target + self.revolute_joints_target)
        rotation = round(random() * calc.RAD360, 5)
        start_point = self.start_points[0]
        # since this is the first joint, we do not need to check solvability
        # but the goal is to move link0 to a specific location
        # so this dimension in the goal space must be narrowed
        if random() < threshold:
            # create prismatic joint
            limit = self._get_random_limit(True)
            self.world.new_link((start_point[0], start_point[1], 0.5), (0, 0, rotation + calc.RAD90),
                                (self.prismatic_length, 1, 1), 'prismatic', upper_limit=limit,
                                create_handle=self.create_handle, joint_axis=(1, 0, 0))
            self.world.create_goal_duplicate((limit, 0, 0))
            self.prismatic_joints_target -= 1
            self._calculate_next_start_point(True, start_point, rotation, limit)
        else:
            # create revolute joint
            limit = self._get_random_limit(False)
            self.world.new_link((start_point[0], start_point[1], 0.5), (0, 0, rotation),
                                (self.revolute_length, 1, 1), 'revolute', auto_limit=limit,
                                create_handle=self.create_handle, hinge_diameter=None)
            self.world.create_goal_duplicate(rotation_offset=(0, 0, limit))
            self.revolute_joints_target -= 1
            self._calculate_next_start_point(False, start_point, rotation, limit)
        self.goal_space_append_with_adjustment((limit, limit))

    def _sample_next_joint(self):
        """
        Assume that the first link+joint as been placed already.
        Try to place a new link+joint at a random position within in a continuous interval so that the puzzle
        1. is UNsolvable if the new link+joint can NOT be moved
        2. is SOLVABLE if the new link+joint CAN be moved
        """
        threshold = self.prismatic_joints_target / (self.prismatic_joints_target + self.revolute_joints_target)
        is_prismatic: bool
        for i in range(self.attempts_per_link):
            offset = random() * self.area_size - self.area_size / 2, random() * self.area_size - self.area_size / 2
            shuffle(self.start_points)
            start_point = self.start_points[0]
            new_point = calc.tuple_add(start_point, offset)
            new_point = round(new_point[0], 5), round(new_point[1], 5)
            rotation = round(random() * calc.RAD360, 5)
            if random() < threshold:
                # create immovable prismatic joint (joint limits = 0)
                self.world.new_link((new_point[0], new_point[1], 0.5), (0, 0, rotation + calc.RAD90),
                                    (self.prismatic_length, 1, 1), 'prismatic', lower_limit=0, upper_limit=0,
                                    create_handle=self.create_handle, joint_axis=(1, 0, 0))
                is_prismatic = True
            else:
                # create immovable revolute joint (joint limits = 0)
                self.world.new_link((new_point[0], new_point[1], 0.5), (0, 0, rotation), (self.revolute_length, 1, 1),
                                    'revolute', lower_limit=0, upper_limit=0, create_handle=self.create_handle,
                                    hinge_diameter=None)
                is_prismatic = False
            self.world.export()
            result = solve(self.world.urdf_path, self.start_state, self.goal_space,
                           self.planning_time * self.first_test_time_multiplier, verbose=False)
            if result == 0:
                # can be solved with the immovable joint
                # we do not want that
                # this new joint should block the previous joint
                self.world.remove_last_object()
                continue
            else:
                # the new (immovable) joint successfully blocks the previously solvable puzzle
                # now make it movable
                limit = self._get_random_limit(is_prismatic)
                self.world.set_limit_of_latest_link(limit, is_prismatic)
                limits_tuple = self.get_limits_tuple(limit)
                self.goal_space_append_with_adjustment(limits_tuple)
                self.start_state.append(0)

                # and check solvability again
                result = solve(self.world.urdf_path, self.start_state, self.goal_space, self.planning_time,
                               verbose=False)
                if result == 0:
                    self._calculate_next_start_point(is_prismatic, new_point, rotation, limit)
                    if is_prismatic:
                        self.prismatic_joints_target -= 1
                    else:
                        self.revolute_joints_target -= 1
                    return 0
                else:
                    self.start_state.pop()
                    self.goal_space.pop()
                    self.world.remove_last_object()

        return 1

    def _clean_up(self):
        self.prismatic_joints_target = self.number_prismatic_joints
        self.revolute_joints_target = self.number_revolute_joints
        self.start_points = [(0, 0)]
        self.start_state = []
        self.goal_space = []
        self.planning_time = self.initial_planning_time

    def _create_continuous_space_puzzle(self):
        """
        Sample links with joints iteratively
        """
        self._sample_first_joint()
        for i in range(1, self.total_number_joints):
            result = self._sample_next_joint()
            if result != 0:
                print("\U000026D4 " * 64)
                print("Could NOT sample link" + str(i), "after", self.attempts_per_link, "attempts!")
                print("\U000026D4 " * 64)
                self._clean_up()
                return result
            print("Successfully sampled link" + str(i), "\U000026F3 " * (i + 1))
            self.planning_time *= self.planning_time_multiplier
        print("\U000026F3 " * 32)
        print("SUCCESS! Sampled", self.total_number_joints, "links!")
        print("\U000026F3 " * 32)
        return 0

    def build(self):
        """Build complete model in Blender and export to URDF. Sample random positions for joints."""
        for i in range(self.attempts):
            self.world.initialize(self.floor_size)
            result = self._create_continuous_space_puzzle()
            progress = round((i + 1) / self.attempts * 100)
            print("Attempt", i + 1, "of", self.attempts, "done [" + ("#" * progress) + (" " * (100 - progress)) + "]")
            if result == 0:
                self.world.render_images()
                return 0
        print("ContinuousSpaceSampler failed!")
        return result


class Lockbox2017Sampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        if "puzzle_name" in config:
            world.update_name(config["puzzle_name"])
        else:
            world.update_name("lockbox2017")
        self.mesh = config["mesh1"]

    def build(self):
        self.world.initialize(self.floor_size)

        self.world.new_door((-6, -1, 1), (0, 0, 0), (2, 0.2, 2))
        self.world.create_goal_duplicate(rotation_offset=(0, 0, calc.RAD90))
        self.start_state.append(0)
        self.goal_space_append_with_adjustment((calc.RAD90, calc.RAD90))

        self.world.new_link((-4, 0, 0.5), (0, 0, 0), (3, 0.5, 1), 'prismatic', 0, 2,
                            create_handle=self.create_handle, joint_axis=(1, 0, 0))
        self.start_state.append(0)
        self.goal_space_append_with_adjustment((0, 2))

        self.world.new_link((0, 0, 0.5), (0, 0, calc.RAD90), (4, 4, 1), 'revolute', 0, calc.RAD90,
                            blend_file=self.mesh, object_name="slot_disc", create_handle=self.create_handle,
                            hinge_diameter=0.25)
        self.start_state.append(0)
        self.goal_space_append_with_adjustment((0, calc.RAD90))

        self.world.new_link((0, 2, 0.5), (0, 0, calc.RAD90), (3, 0.5, 1), 'prismatic', 0, 2,
                            create_handle=self.create_handle, joint_axis=(1, 0, 0))
        self.start_state.append(0)
        self.goal_space_append_with_adjustment((0, 2))

        self.world.new_door((-2, 4, 1), (0, 0, 0), (4, 0.2, 2))
        self.start_state.append(0)
        self.goal_space_append_with_adjustment((0, calc.RAD90))

        self.world.export(concave_collision_mesh=True)
        self.world.render_images()
        return 0


class LockboxRandomSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        if "puzzle_name" in config:
            world.update_name(config["puzzle_name"])
        else:
            world.update_name("lockbox_random")
        self.mesh = config["mesh1"]
        self.iterations = config["iterations"]
        self.slider_length = config["slider_length"]
        self.slider_width = config["slider_width"]
        self.radius_interval = config["radius_interval"]
        self.previous_direction = ""
        self.available_directions = {
            "N": ["N", "E", "W"],
            "E": ["N", "E", "S"],
            "S": ["E", "S", "W"],
            "W": ["N", "S", "W"],
        }
        self.get_rotation = {
            "N": calc.RAD90,
            "E": 0,
            "S": -calc.RAD90,
            "W": calc.RAD180,
        }

    def add_slot_disc_and_slider(self, direction):
        radius = self.radius_interval[0] + random() * (self.radius_interval[1] - self.radius_interval[0])
        rotation = self.get_rotation[direction]

        offset_start = calc.tuple_scale(calc.DIRECTION_VECTOR_2D[self.previous_direction], radius + 1)
        start_point = calc.tuple_add(self.start_point, offset_start)

        slot_disc_location = (start_point[0], start_point[1], 0.5)
        self.world.new_link(slot_disc_location, (0, 0, rotation), (radius * 2, radius * 2, 1), 'revolute',
                            -calc.RAD180, calc.RAD180, blend_file=self.mesh, object_name="slot_disc",
                            create_handle=self.create_handle, hinge_diameter=radius * 0.125)

        offset_slider = calc.tuple_scale(calc.DIRECTION_VECTOR_2D[direction], radius)
        slider_location = (start_point[0] + offset_slider[0], start_point[1] + offset_slider[1], 0.5)
        self.world.new_link(slider_location, (0, 0, rotation), (self.slider_length, self.slider_width, 1),
                            'prismatic', 0, 1, create_handle=self.create_handle, joint_axis=(1, 0, 0))
        return slider_location

    def choose_links(self):
        directions = self.available_directions[self.previous_direction]
        shuffle(directions)
        for random_direction in directions:
            new_start = self.add_slot_disc_and_slider(random_direction)
            self.start_state.append(0)
            self.start_state.append(0)
            self.goal_space.append((-calc.RAD180, calc.RAD180))
            self.goal_space.append((0, 1))
            self.world.export(concave_collision_mesh=True)
            if solve(self.world.urdf_path, self.start_state, None, only_check_start_state_validity=True) == 0:
                self.start_point = new_start
                self.previous_direction = random_direction
                return 0
            else:
                self.world.remove_last_object()
                self.world.remove_last_object()
                self.start_state.pop()
                self.start_state.pop()
                self.goal_space.pop()
                self.goal_space.pop()
        return 1

    def build(self):
        self.world.initialize(self.floor_size)

        # first slider
        self.previous_direction = choice(("N", "E", "S", "W"))
        rotation = self.get_rotation[self.previous_direction]
        self.world.new_link((self.start_point[0], self.start_point[1], 0.5), (0, 0, rotation),
                            (self.slider_length, self.slider_width, 1), 'prismatic', 0, 1,
                            create_handle=self.create_handle, joint_axis=(1, 0, 0))
        self.world.create_goal_duplicate((1, 0, 0))
        self.start_state.append(0)
        self.goal_space.append((1, 1))

        for i in range(self.iterations):
            if self.choose_links() == 1:
                print("LockboxRandomSampler failed!")
                return 1

        self.world.render_images()
        return 0


class EscapeRoomSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        if "puzzle_name" in config:
            world.update_name(config["puzzle_name"])
        else:
            world.update_name("escape_room")
        self.robot_mesh = config["robot_mesh"]

    def build(self):
        self.world.initialize(self.floor_size)

        # add robot
        first = self.world.new_link((0, 0, 0.5), (0, 0, 0), (0, 0, 0), 'prismatic', -6, 6, joint_axis=(1, 0, 0))
        second = self.world.new_link((0, 0, 0), (0, 0, 0), (0, 0, 0), 'prismatic', -6, 6, joint_axis=(0, 1, 0),
                                     parent=first)
        robot = self.world.new_link((0, 0, 0), (0, 0, 0), (0.75, 1, 1), 'revolute', -calc.RAD180, calc.RAD180,
                                    parent=second, blend_file="input-meshes/droids.blend", object_name="droids_3",
                                    new_mesh_name="robot")
        self.start_state.extend((0, 0, 0))
        self.goal_space.extend(((0, 0), (5, 5)))
        self.goal_space_append_with_adjustment((calc.RAD90, calc.RAD90))

        self.world.create_goal_duplicate((0, 5, 0), (0, 0, calc.RAD90))

        # add obstacle
        y = random() * 2 + 1
        first = self.world.new_link((-0.75, y, 0.25), (0, 0, 0), (0, 0, 0), 'prismatic', -6, 6, joint_axis=(1, 0, 0))
        second = self.world.new_link((0, 0, 0), (0, 0, 0), (0, 0, 0), 'prismatic', -6, 6, joint_axis=(0, 1, 0),
                                     parent=first)
        obstacle = self.world.new_link((0, 0, 0), (0, 0, 0), (1.5, 0.5, 0.5), 'revolute', -calc.RAD180, calc.RAD180,
                                       parent=second, blend_file="input-meshes/l1_stick.blend", object_name="l1_stick",
                                       new_mesh_name="stick", material=color.BROWN)
        self.start_state.extend((0, 0, 0))
        self.goal_space.extend(((-16, 16), (-16, 16)))
        self.goal_space_append_with_adjustment((-calc.RAD180, calc.RAD180))

        # add door
        self.world.new_door((1.5, 4, 0.5), (0, 0, calc.RAD180), (3.2, 0.2, 1), top_handle=False)
        self.start_state.append(0)
        self.goal_space_append_with_adjustment((0, calc.RAD90))

        # add walls
        self.world.new_link((0, -4, 0.25), (0, 0, 0), (3.8, 0.2, 0.5), name="wall_left", material=color.GRAY)
        self.world.new_link((2, 0, 0.25), (0, 0, 0), (0.2, 8, 0.5), name="wall_front", material=color.GRAY)
        self.world.new_link((-2, 0, 0.25), (0, 0, 0), (0.2, 8, 0.5), name="wall_back", material=color.GRAY)

        self.world.export()
        self.world.render_images()
        return 0


class MoveTwiceSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        if "puzzle_name" in config:
            world.update_name(config["puzzle_name"])
        else:
            world.update_name("move_twice")
        self.robot_mesh = config["robot_mesh"]

    def build(self):
        self.world.initialize(self.floor_size)

        first = self.world.new_link((0, 0, 0.5), (0, 0, 0), (0, 0, 0), 'prismatic', -16, 16, joint_axis=(1, 0, 0))
        second = self.world.new_link((0, 0, 0), (0, 0, 0), (0, 0, 0), 'prismatic', -16, 16, joint_axis=(0, 1, 0),
                                     parent=first)
        robot = self.world.new_link((0, 0, 0), (0, 0, 0), (0.75, 1, 1), 'revolute', -calc.RAD180, calc.RAD180,
                                    parent=second, blend_file="input-meshes/droids.blend", object_name="droids_3",
                                    new_mesh_name="robot")

        start = (0, 0, 0)
        # start = (random() * 3 - 1.5, random() * 1.4 - 0.7, random() * calc.RAD360 - calc.RAD180)
        self.start_state.extend(start)

        goal = (random() * 3 - 1.5, random() * 1.4 - 0.7 + 3, random() * calc.RAD360 - calc.RAD180)
        self.goal_space_append_with_adjustment((goal[0], goal[0]))
        self.goal_space_append_with_adjustment((goal[1], goal[1]))
        self.goal_space_append_with_adjustment((goal[2], goal[2]))

        self.world.create_goal_duplicate((goal[0], goal[1], 0), (0, 0, goal[2]))

        self.world.new_link((0, -1.4, 0.5), (0, 0, 0), (4, 0.2, 1), name="wall_left", material=color.GRAY)
        self.world.new_link((0, 4.4, 0.5), (0, 0, 0), (4, 0.2, 1), name="wall_right", material=color.GRAY)
        self.world.new_link((0, 1.5, 0.5), (0, 0, 0), (4, 0.2, 1), name="wall_mid", material=color.GRAY)
        self.world.new_link((-2.1, 1.5, 0.5), (0, 0, 0), (0.2, 6, 1), name="wall_back", material=color.GRAY)
        self.world.new_link((2.1, 1.5, 0.5), (0, 0, 0), (0.2, 6, 1), 'prismatic', -3, 3, joint_axis=(0, 1, 0))
        self.goal_space.append((-2, 2))
        self.start_state.append(0)

        self.world.export()
        self.world.render_images()
        return 0


class MoveNTimesSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        if "puzzle_name" in config:
            world.update_name(config["puzzle_name"])
        else:
            world.update_name("move_n_times")
        self.robot_mesh = config["robot_mesh"]
        self.n = config["n"]

    def build(self):
        self.world.initialize(self.floor_size)
        if self.n == 0:
            n = 1
        else:
            n = self.n

        first = self.world.new_link((0, 0, 0.25), (0, 0, 0), (0, 0, 0), 'prismatic', -n, n, joint_axis=(1, 0, 0))
        second = self.world.new_link((0, 0, 0), (0, 0, 0), (0, 0, 0), 'prismatic', -n, 0.5, joint_axis=(0, 1, 0),
                                     parent=first)
        robot = self.world.new_link((0, 0, 0), (0, 0, 0), (0.375, 0.5, 0.5), 'revolute', -calc.RAD180, calc.RAD180,
                                    parent=second, blend_file="input-meshes/droids.blend", object_name="droids_3",
                                    new_mesh_name="robot")
        start = (0, 0, 0)
        self.start_state.extend(start)
        goal = (random() * n * 2 - n, -n, random() * calc.RAD360 - calc.RAD180)
        self.goal_space_append_with_adjustment((goal[0], goal[0]))
        self.goal_space_append_with_adjustment((goal[1], goal[1]))
        self.goal_space_append_with_adjustment((goal[2], goal[2]))
        self.world.create_goal_duplicate((goal[0], goal[1], 0), (0, 0, goal[2]))

        self.world.link_offset = (0.5, -0.5, 0.25)

        if n > 2:
            width = n * 2 - 2
        else:
            width = 2
        limit = width / 2
        self.world.new_link((0, 0, 0), (0, 0, 0), (width, 0.1, 0.5), 'prismatic', -limit, limit,
                            joint_axis=(1, 0, 0))
        self.goal_space.append((-limit, limit))
        self.start_state.append(0)

        if self.n > 0:
            self.world.new_link((-0.5, 1, 0), (0, 0, 0), (0.9, 0.1, 0.5), name="wall_north_0", material=color.GRAY)
            self.world.new_link((-1, 0.5, 0), (0, 0, 0), (0.1, 0.9, 0.5), name="wall_northwest_0", material=color.GRAY)
            self.world.new_link((0, 0.5, 0), (0, 0, 0), (0.1, 0.9, 0.5), name="wall_northeast_0", material=color.GRAY)

        for i in range(1, n):
            self.world.new_link((0, -i, 0), (0, 0, 0), (i * 2 - 0.1, 0.1, 0.5), name="wall_south_" + str(i),
                                material=color.GRAY)
            self.world.new_link((-i, -i / 2, 0), (0, 0, 0), (0.1, i - 0.1, 0.5), name="wall_southwest_" + str(i),
                                material=color.GRAY)
            self.world.new_link((i, -i / 2, 0), (0, 0, 0), (0.1, i - 0.1, 0.5), name="wall_southeast_" + str(i),
                                material=color.GRAY)

            if i > 1:
                if i % 2 == 0:
                    x1 = i - 1
                    x2 = i
                else:
                    x1 = -(i - 1)
                    x2 = -i
                self.world.new_link((x1, 1, 0), (0, 0, 0), (1.9, 0.1, 0.5), name="wall_north_" + str(i),
                                    material=color.GRAY)
                self.world.new_link((x2, 0.5, 0), (0, 0, 0), (0.1, 0.9, 0.5), name="wall_north_side_" + str(i),
                                    material=color.GRAY)

        self.world.export()
        self.world.render_images()
        return 0
