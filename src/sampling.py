from random import seed, random, choice
import os
import sys

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from world import BlenderWorld
from solvability_testing import test_urdf
import calc


class PuzzleSampler:
    def __init__(self, config, world: BlenderWorld):
        self.world = world
        self.floor_size = config["floor_size"]
        self.number_prismatic_joints = config["number_prismatic_joints"]
        self.number_revolute_joints = config["number_revolute_joints"]
        self.total_number_joints = self.number_prismatic_joints + self.number_revolute_joints
        self.branching_factor = config["branching_factor_target"]
        self.attempts = config["attempts"]
        seed(config["seed_for_randomness"])
        self.prismatic_joints_target = self.number_prismatic_joints
        self.revolute_joints_target = self.number_revolute_joints
        self.branching_target = self.branching_factor
        self.start_point = (0, 0)
        self.prismatic_length = 2
        self.revolute_length = 3
        self.goal_adjustment = 0.0001
        self.start_state = []
        self.goal_space = []

    def build(self):
        raise NotImplementedError

    def goal_space_append(self, limits: tuple):
        if len(limits) != 2:
            print("goal_space_append(): INPUT TUPLE LENGTH IS NOT 2")
        lower_limit = limits[0]
        upper_limit = limits[1]
        if abs(upper_limit - lower_limit) > 2 * self.goal_adjustment:
            if lower_limit != 0:
                lower_limit += self.goal_adjustment
            if upper_limit != 0:
                upper_limit -= self.goal_adjustment
        else:
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

    def return_lower_and_upper_limit(self, span):
        if span < 0:
            return span, 0
        else:
            return 0, span


class SimpleSlidersSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        world.update_name("simple_sliders")

    def _create_simple_sliders_puzzle(self):
        """
        Create a very simple model that only works with prismatic joints
        (ignore number_revolute_joints and branching_factor).
        """
        for i in range(self.number_prismatic_joints):
            if i % 2 == 0:
                self.world.new_object(location=(i / 2, i / -2, 0.1), rotation=(calc.RAD90, 0, 0),
                                      scale=(0.2, 0.2, 1.6), joint_type='prismatic', upper_limit=1)
            else:
                self.world.new_object(location=((i - 1) / 2, ((i - 1) / -2) - 1, 0.1), rotation=(0, calc.RAD90, 0),
                                      scale=(0.2, 0.2, 1.6), joint_type='prismatic', upper_limit=1)
            self.goal_space_append((0, 1))

    def build(self):
        """Build complete model in Blender and export to URDF. Create only prismatic joints."""
        self.start_state = [0] * self.number_prismatic_joints
        self.world.reset()
        self.world.create_base_link(self.floor_size)
        self._create_simple_sliders_puzzle()
        self.world.create_collision()
        self.world.export()
        return 0


class GridWorldSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        world.update_name("grid_world")
        self.allow_clockwise = config["allow_clockwise"]
        self.epsilon = config["epsilon"]
        self.start_points = [(0.5, 0.5)]
        self.occupied_fields = self.start_points.copy()
        self.position_sequence = []

    def _new_prismatic_joint(self):
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
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, 2)))
            self.start_points.append(calc.tuple_add(sp, (0, 2)))
        elif random_pos == "E":
            # add new prismatic joint at this position
            loc = (sp[0] + 0.5, sp[1], scale[0] / 2)
            rot = (0, calc.RAD90, 0)
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (2, 0)))
            self.start_points.append(calc.tuple_add(sp, (2, 0)))
        elif random_pos == "S":
            # add new prismatic joint at this position
            loc = (sp[0], sp[1] - 0.5, scale[0] / 2)
            rot = (calc.RAD90, 0, 0)
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (0, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, -2)))
            self.start_points.append(calc.tuple_add(sp, (0, -2)))
        elif random_pos == "W":
            # add new prismatic joint at this position
            loc = (sp[0] - 0.5, sp[1], scale[0] / 2)
            rot = (0, -calc.RAD90, 0)
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='prismatic', upper_limit=1)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (-2, 0)))
            self.start_points.append(calc.tuple_add(sp, (-2, 0)))

        # update target counter
        self.prismatic_joints_target -= 1

        return 0

    def _new_revolute_joint(self):
        # check available positions for revolute joint
        positions = []
        sp = self.start_points.pop(0)
        of = self.occupied_fields
        if (calc.tuple_add(sp, (0, 1)) not in of and calc.tuple_add(sp, (0, 2)) not in of
                and calc.tuple_add(sp, (1, 1)) not in of and calc.tuple_add(sp, (-1, 1)) not in of):
            # North
            if calc.tuple_add(sp, (-1, 2)) not in of and calc.tuple_add(sp, (1, 0)) not in of:
                positions.append("N_counterclockwise")
            if calc.tuple_add(sp, (1, 2)) not in of and calc.tuple_add(sp, (-1, 0)) not in of and self.allow_clockwise:
                positions.append("N_clockwise")
        if (calc.tuple_add(sp, (1, 0)) not in of and calc.tuple_add(sp, (2, 0)) not in of
                and calc.tuple_add(sp, (1, 1)) not in of and calc.tuple_add(sp, (1, -1)) not in of):
            # East
            if calc.tuple_add(sp, (2, 1)) not in of and calc.tuple_add(sp, (0, -1)) not in of:
                positions.append("E_counterclockwise")
            if calc.tuple_add(sp, (0, 1)) not in of and calc.tuple_add(sp, (2, -1)) not in of and self.allow_clockwise:
                positions.append("E_clockwise")
        if (calc.tuple_add(sp, (0, -1)) not in of and calc.tuple_add(sp, (0, -2)) not in of
                and calc.tuple_add(sp, (-1, -1)) not in of and calc.tuple_add(sp, (1, -1)) not in of):
            # South
            if calc.tuple_add(sp, (-1, 0)) not in of and calc.tuple_add(sp, (1, -2)) not in of:
                positions.append("S_counterclockwise")
            if calc.tuple_add(sp, (1, 0)) not in of and calc.tuple_add(sp, (-1, -2)) not in of and self.allow_clockwise:
                positions.append("S_clockwise")
        if (calc.tuple_add(sp, (-1, 0)) not in of and calc.tuple_add(sp, (-2, 0)) not in of
                and calc.tuple_add(sp, (-1, 1)) not in of and calc.tuple_add(sp, (-1, -1)) not in of):
            # West
            if calc.tuple_add(sp, (0, 1)) not in of and calc.tuple_add(sp, (-2, -1)) not in of:
                positions.append("W_counterclockwise")
            if calc.tuple_add(sp, (-2, 1)) not in of and calc.tuple_add(sp, (0, -1)) not in of and self.allow_clockwise:
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
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute',
                                  upper_limit=calc.RAD90)
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
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute',
                                  lower_limit=-calc.RAD90)
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
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute',
                                  upper_limit=calc.RAD90)
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
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute',
                                  lower_limit=-calc.RAD90)
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
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute',
                                  upper_limit=calc.RAD90)
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
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute',
                                  lower_limit=-calc.RAD90)
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
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute',
                                  upper_limit=calc.RAD90)
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
            self.world.new_object(location=loc, rotation=rot, scale=scale, joint_type='revolute',
                                  lower_limit=-calc.RAD90)
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

    def _new_joint(self, try_prismatic_first):
        result = 1
        if try_prismatic_first:
            # try creating a prismatic joint
            result = self._new_prismatic_joint()
            if result != 0 and self.revolute_joints_target != 0:
                # if creating a prismatic joint failed and revolute joints are still needed,
                # try creating a revolute joint
                result = self._new_revolute_joint()

        else:
            # try creating a revolute joint
            result = self._new_revolute_joint()
            if result != 0 and self.prismatic_joints_target != 0:
                # if creating a revolute joint failed and prismatic joints are still needed,
                # try creating a prismatic joint
                result = self._new_prismatic_joint()

        return result

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
            result = self._new_joint(try_prismatic)
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

    def build(self):
        """Build complete model in Blender and export to URDF."""
        self.start_state = [0] * self.total_number_joints
        result = 1
        while result != 0:
            self.world.reset()
            if self.attempts <= 0:
                return result
            self.world.create_base_link(self.floor_size)
            result = self._create_grid_world_puzzle()
            self.attempts -= 1

        self.world.create_collision()
        self.world.export()
        return 0


class ContinuousSpaceSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        world.update_name("continuous_space")
        self.planning_time = config["start_planning_time"]
        self.planning_time_multiplier = config["planning_time_multiplier"]
        self.first_test_time_multiplier = config["first_test_time_multiplier"]
        self.area_size = config["area_size"]
        self.upper_limit_prismatic = config["upper_limit_prismatic"]
        self.upper_limit_revolute = config["upper_limit_revolute"]

    def _get_random_limit_span(self, is_prismatic):
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

    def _calculate_next_start_point(self, is_prismatic, pos, rotation, limit):
        """
        Calculate the center of the sampling area for the next joint to maximize the chance of blocking the previous
        joint.
        """
        if is_prismatic:
            link_oriented = calc.rotate((0, self.prismatic_length / 2 + limit / 2), rotation)
        else:
            link_oriented = calc.rotate((self.revolute_length, 0), rotation + calc.RAD90)
        return calc.tuple_add(pos, link_oriented)

    def _sample_first_joint(self):
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
            limit_span = self._get_random_limit_span(True)
            self.world.new_object((self.start_point[0], self.start_point[1], 0.5), (-calc.RAD90, 0, rotation),
                                  (1, 1, self.prismatic_length), 'prismatic', lower_limit=0, upper_limit=limit_span)
            self.prismatic_joints_target -= 1
            self.start_point = self._calculate_next_start_point(True, self.start_point, rotation, limit_span)
        else:
            # create revolute joint
            limit_span = self._get_random_limit_span(False)
            if limit_span > 0:
                self.world.new_object((self.start_point[0], self.start_point[1], 0.5), (0, 0, rotation),
                                      (self.revolute_length, 1, 1), 'revolute', lower_limit=0, upper_limit=limit_span)
            else:
                self.world.new_object((self.start_point[0], self.start_point[1], 0.5), (0, 0, rotation),
                                      (self.revolute_length, 1, 1), 'revolute', lower_limit=limit_span, upper_limit=0)
            self.revolute_joints_target -= 1
            self.start_point = self._calculate_next_start_point(False, self.start_point, rotation, limit_span)
        self.goal_space_append((limit_span, limit_span))

    def _sample_next_joint(self):
        """
        Assume that the first link+joint as been placed already.
        Try to place a new link+joint at a random position within in a continuous interval so that the puzzle
        1. is UNsolvable if the new link+joint can NOT be moved
        2. is SOLVABLE if the new link+joint CAN be moved
        """
        threshold = self.prismatic_joints_target / (self.prismatic_joints_target + self.revolute_joints_target)
        is_prismatic: bool
        for i in range(self.attempts):
            offset = (random() * self.area_size - self.area_size / 2, random() * self.area_size - self.area_size / 2)
            new_point = calc.tuple_add(self.start_point, offset)
            rotation = random() * calc.RAD360
            if random() < threshold:
                # create immovable prismatic joint (joint limits = 0)
                self.world.new_object((new_point[0], new_point[1], 0.5), (-calc.RAD90, 0, rotation),
                                      (1, 1, self.prismatic_length), 'prismatic', lower_limit=0, upper_limit=0)
                is_prismatic = True
            else:
                # create immovable revolute joint (joint limits = 0)
                self.world.new_object((new_point[0], new_point[1], 0.5), (0, 0, rotation), (self.revolute_length, 1, 1),
                                      'revolute', lower_limit=0, upper_limit=0)
                is_prismatic = False
            self.world.create_collision(self.world.movable_visual_objects[-1])
            self.world.export()
            result = test_urdf(self.world.urdf_path, self.start_state, self.goal_space,
                               self.planning_time * self.first_test_time_multiplier)
            if result == 0:
                # can be solved with the immovable joint
                # we do not want that
                # this new joint should block the previous joint
                self.world.remove_last_object()
                continue
            else:
                # the new (immovable) joint successfully blocks the previously solvable puzzle
                # now make it movable
                limit_span = self._get_random_limit_span(is_prismatic)
                self.world.set_limit_of_active_object(limit_span, is_prismatic)
                limits_tuple = self.return_lower_and_upper_limit(limit_span)
                self.goal_space_append(limits_tuple)
                self.start_state.append(0)

                # and check solvability again
                result = test_urdf(self.world.urdf_path, self.start_state, self.goal_space, self.planning_time)
                if result == 0:
                    self.start_point = self._calculate_next_start_point(is_prismatic, new_point, rotation, limit_span)
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

    def _create_continuous_space_puzzle(self):
        """
        Sample links with joints iteratively
        """
        self._sample_first_joint()
        self.world.create_collision()
        for i in range(1, self.total_number_joints):
            result = self._sample_next_joint()
            if result != 0:
                print("\U000026D4 " * 64)
                print("Could NOT sample link" + str(i), "after", self.attempts, "attempts!")
                print("\U000026D4 " * 64)
                return result
            print("Successfully sampled link" + str(i), "\U000026F3 " * i)
            self.planning_time *= self.planning_time_multiplier
        print("\U000026F3 " * 32)
        print("SUCCESS! Sampled", self.total_number_joints, "links!")
        print("\U000026F3 " * 32)
        return 0

    def build(self):
        """Build complete model in Blender and export to URDF. Sample random positions for joints."""
        self.world.reset()
        self.world.create_base_link(self.floor_size)
        result = self._create_continuous_space_puzzle()
        if result == 0:
            return 0
        else:
            return result


class Lockbox2017Sampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)

    def build(self):
        raise NotImplementedError
