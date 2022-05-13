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
        self.branching_factor = config["branching_factor_target"]
        self.attempts = config["attempts"]
        seed(config["seed_for_randomness"])
        self.create_handle = config["create_handle"]
        self.prismatic_joints_target = self.number_prismatic_joints
        self.revolute_joints_target = self.number_revolute_joints
        self.branching_target = self.branching_factor
        self.start_point = (0, 0)
        self.prismatic_length = 2
        self.revolute_length = 3
        self.goal_adjustment = 0.  # e.g. 0.0001
        self.start_state = []
        self.goal_space = []

    def build(self):
        raise NotImplementedError

    def goal_space_append(self, limits: tuple):
        if len(limits) != 2:
            print("goal_space_append(): INPUT TUPLE LENGTH IS NOT 2")
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

    def return_lower_and_upper_limit(self, span):
        if span < 0:
            return span, 0
        else:
            return 0, span


class SimpleSlidersSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        if "puzzle_name" in config:
            world.update_name(config["puzzle_name"])
        else:
            world.update_name("simple_sliders")

    def _create_simple_sliders_puzzle(self):
        """
        Create a very simple model that only works with prismatic joints
        (ignore number_revolute_joints and branching_factor).
        """
        scale = (0.2, 0.2, 1.6)
        for i in range(self.number_prismatic_joints):
            if i % 2 == 0:
                location = (i / 2, i / -2, 0.1)
                rotation = (calc.RAD90, 0, 0)
            else:
                location = ((i - 1) / 2, ((i - 1) / -2) - 1, 0.1)
                rotation = (0, calc.RAD90, 0)
            self.world.new_link(location, rotation, scale, 'prismatic', upper_limit=1, create_handle=self.create_handle)
            self.goal_space.append((0, 1))
        self.goal_space_narrow(dimension=0)
        self.start_state = [0] * self.number_prismatic_joints
        self.world.create_goal_duplicate((0, 0, 1))

    def build(self):
        """Build complete model in Blender and export to URDF. Create only prismatic joints."""
        self.world.reset()
        self.world.create_base_link(self.floor_size)
        self._create_simple_sliders_puzzle()
        self.world.export()
        self.world.render_image()
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
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (0, 1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, 2)))
            self.start_points.append(calc.tuple_add(sp, (0, 2)))
        elif random_pos == "E":
            # add new prismatic joint at this position
            loc = (sp[0] + 0.5, sp[1], scale[0] / 2)
            rot = (0, calc.RAD90, 0)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (2, 0)))
            self.start_points.append(calc.tuple_add(sp, (2, 0)))
        elif random_pos == "S":
            # add new prismatic joint at this position
            loc = (sp[0], sp[1] - 0.5, scale[0] / 2)
            rot = (calc.RAD90, 0, 0)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (0, -1)))
            self.occupied_fields.append(calc.tuple_add(sp, (0, -2)))
            self.start_points.append(calc.tuple_add(sp, (0, -2)))
        elif random_pos == "W":
            # add new prismatic joint at this position
            loc = (sp[0] - 0.5, sp[1], scale[0] / 2)
            rot = (0, -calc.RAD90, 0)
            # update occupied fields and start point
            self.occupied_fields.append(calc.tuple_add(sp, (-1, 0)))
            self.occupied_fields.append(calc.tuple_add(sp, (-2, 0)))
            self.start_points.append(calc.tuple_add(sp, (-2, 0)))

        self.world.new_link(calc.tuple_scale(loc, self.scaling), rot, calc.tuple_scale(scale, self.scaling),
                            'prismatic', upper_limit=1 * self.scaling, create_handle=self.create_handle)

        # update goal_space target counter
        self.goal_space.append((0, 1 * self.scaling))
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
            limit = calc.RAD90
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
            limit = -calc.RAD90
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
            limit = calc.RAD90
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
            limit = -calc.RAD90
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
            limit = calc.RAD90
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
            limit = -calc.RAD90
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
            limit = calc.RAD90
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
            limit = -calc.RAD90
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

        self.world.new_link(calc.tuple_scale(loc, self.scaling), rot, calc.tuple_scale(scale, self.scaling), 'revolute',
                            auto_limit=limit, create_handle=self.create_handle)
        self.goal_space_append(self.return_lower_and_upper_limit(limit))

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
                self.start_points = [(0.5, 0.5)]
                self.world.movable_links = []
                return result
        print("SUCCESSFULLY CREATED THE FOLLOWING SEQUENCE: " + str(self.position_sequence))
        self.goal_space_narrow(dimension=0)
        goal_limit = self.goal_space[0][1]
        if len(self.position_sequence[0]) == 1:
            self.world.create_goal_duplicate((0, 0, goal_limit))
        else:
            self.world.create_goal_duplicate(rotation_offset=(0, 0, goal_limit))

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

        self.world.export()
        self.world.render_image()
        return 0


class ContinuousSpaceSampler(PuzzleSampler):
    def __init__(self, config, world: BlenderWorld):
        super().__init__(config, world)
        if "puzzle_name" in config:
            world.update_name(config["puzzle_name"])
        else:
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
        rotation = round(random() * calc.RAD360, 5)
        # since this is the first joint, we do not need to check solvability
        # but the goal is to move link0 to a specific location
        # so this dimension in the goal space must be narrowed
        if random() < threshold:
            # create prismatic joint
            limit_span = self._get_random_limit_span(True)
            self.world.new_link((self.start_point[0], self.start_point[1], 0.5), (-calc.RAD90, 0, rotation),
                                (1, 1, self.prismatic_length), 'prismatic', lower_limit=0, upper_limit=limit_span,
                                create_handle=self.create_handle)
            self.world.create_goal_duplicate((0, 0, limit_span))
            self.prismatic_joints_target -= 1
            self.start_point = self._calculate_next_start_point(True, self.start_point, rotation, limit_span)
        else:
            # create revolute joint
            limit_span = self._get_random_limit_span(False)
            if limit_span > 0:
                self.world.new_link((self.start_point[0], self.start_point[1], 0.5), (0, 0, rotation),
                                    (self.revolute_length, 1, 1), 'revolute', lower_limit=0, upper_limit=limit_span,
                                    create_handle=self.create_handle)
            else:
                self.world.new_link((self.start_point[0], self.start_point[1], 0.5), (0, 0, rotation),
                                    (self.revolute_length, 1, 1), 'revolute', lower_limit=limit_span, upper_limit=0,
                                    create_handle=self.create_handle)
            self.world.create_goal_duplicate(rotation_offset=(0, 0, limit_span))
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
            offset = random() * self.area_size - self.area_size / 2, random() * self.area_size - self.area_size / 2
            new_point = calc.tuple_add(self.start_point, offset)
            new_point = round(new_point[0], 5), round(new_point[1], 5)
            rotation = round(random() * calc.RAD360, 5)
            if random() < threshold:
                # create immovable prismatic joint (joint limits = 0)
                self.world.new_link((new_point[0], new_point[1], 0.5), (-calc.RAD90, 0, rotation),
                                    (1, 1, self.prismatic_length), 'prismatic', lower_limit=0, upper_limit=0,
                                    create_handle=self.create_handle)
                is_prismatic = True
            else:
                # create immovable revolute joint (joint limits = 0)
                self.world.new_link((new_point[0], new_point[1], 0.5), (0, 0, rotation), (self.revolute_length, 1, 1),
                                      'revolute', lower_limit=0, upper_limit=0, create_handle=self.create_handle)
                is_prismatic = False
            self.world.export()
            result = solve(self.world.urdf_path, self.start_state, self.goal_space,
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
                self.world.set_limit_of_latest_link(limit_span, is_prismatic)
                limits_tuple = self.return_lower_and_upper_limit(limit_span)
                self.goal_space_append(limits_tuple)
                self.start_state.append(0)

                # and check solvability again
                result = solve(self.world.urdf_path, self.start_state, self.goal_space, self.planning_time)
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
        for i in range(1, self.total_number_joints):
            result = self._sample_next_joint()
            if result != 0:
                print("\U000026D4 " * 64)
                print("Could NOT sample link" + str(i), "after", self.attempts, "attempts!")
                print("\U000026D4 " * 64)
                return result
            print("Successfully sampled link" + str(i), "\U000026F3 " * (i + 1))
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
            self.world.render_image()
            return 0
        else:
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
        self.world.reset()
        self.world.create_base_link(self.floor_size)

        self.world.new_door((-6, -1, 1), (0, 0, 0), (2, 0.2, 2))
        self.world.create_goal_duplicate(rotation_offset=(0, 0, calc.RAD90))
        self.start_state.append(0)
        self.goal_space_append((calc.RAD90, calc.RAD90))

        self.world.new_link((-4, 0, 0.5), (-calc.RAD90, 0, -calc.RAD90), (0.8, 1, 3.6), 'prismatic', 0, 2,
                            create_handle=self.create_handle)
        self.start_state.append(0)
        self.goal_space_append((0, 2))

        self.world.new_link((0, 0, 0.5), (0, 0, calc.RAD90), (4, 4, 1), 'revolute', 0, calc.RAD90,
                            mesh_filepath=self.mesh, object_name="slot_disc", create_handle=self.create_handle)
        self.start_state.append(0)
        self.goal_space_append((0, calc.RAD90))

        self.world.new_link((0, 2, 0.5), (-calc.RAD90, 0, 0), (0.8, 1, 3.6), 'prismatic', 0, 2,
                            create_handle=self.create_handle)
        self.start_state.append(0)
        self.goal_space_append((0, 2))

        self.world.new_door((-2, 4, 1), (0, 0, 0), (4, 0.2, 2))
        self.start_state.append(0)
        self.goal_space_append((0, calc.RAD90))

        self.world.export()
        self.world.render_image(rotation=(1.05, 0, calc.RAD90))
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
        self.previous_direction = "E"

    def add_slot_disc_and_slider(self, direction):
        location_slot_disc = (self.start_point[0], self.start_point[1], 0.5)
        if direction == "N":
            rotation = calc.RAD90
            location_slider = (self.start_point[0], self.start_point[1] + 1.5, 0.5)
            self.start_point = calc.tuple_add(self.start_point, (0, 4))
        elif direction == "E":
            rotation = 0
            location_slider = (self.start_point[0] + 1.5, self.start_point[1], 0.5)
            self.start_point = calc.tuple_add(self.start_point, (4, 0))
        elif direction == "S":
            rotation = -calc.RAD90
            location_slider = (self.start_point[0], self.start_point[1] - 1.5, 0.5)
            self.start_point = calc.tuple_add(self.start_point, (0, -4))
        else:  # direction == "W"
            rotation = calc.RAD180
            location_slider = (self.start_point[0] - 1.5, self.start_point[1], 0.5)
            self.start_point = calc.tuple_add(self.start_point, (-4, 0))

        self.world.new_link(location_slot_disc, (0, 0, rotation), (3, 3, 1), 'revolute', -calc.RAD180, calc.RAD180,
                            mesh_filepath=self.mesh, object_name="slot_disc", create_handle=self.create_handle)
        self.world.new_link(location_slider, (0, calc.RAD90, rotation), (1, self.slider_width, self.slider_length),
                            'prismatic', 0, 1, create_handle=self.create_handle)

    def available_directions(self):
        if self.previous_direction == "N":
            return ["N", "E", "W"]
        elif self.previous_direction == "E":
            return ["N", "E", "S"]
        elif self.previous_direction == "S":
            return ["E", "S", "W"]
        else:  # self.previous_direction == "W"
            return ["N", "S", "W"]

    def choose_links(self):
        directions = self.available_directions()
        shuffle(directions)
        for random_direction in directions:
            self.add_slot_disc_and_slider(random_direction)
            self.start_state.append(0)
            self.start_state.append(0)
            self.goal_space.append((-calc.RAD180, calc.RAD180))
            self.goal_space.append((0, 1))
            self.world.export()
            if solve(self.world.urdf_path, self.start_state, None, only_check_start_state_validity=True) == 0:
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
        self.start_point = (0.5, 0.5)
        self.world.reset()
        self.world.create_base_link(self.floor_size)

        # first slider
        self.world.new_link((self.start_point[0] - 2.5, self.start_point[1], 0.5), (0, calc.RAD90, 0),
                            (1, self.slider_width, self.slider_length), 'prismatic', 0, 1,
                            create_handle=self.create_handle)
        self.world.create_goal_duplicate((0, 0, 1))
        self.start_state.append(0)
        self.goal_space.append((1, 1))

        for i in range(self.iterations):
            if self.choose_links() == 1:
                return 1

        self.world.render_image()
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
        self.world.reset()
        self.world.create_base_link(self.floor_size)

        # add robot
        first = self.world.new_link((0, 0, 0.5), (0, 0, 0), (0, 0, 0), 'prismatic', -6, 6, joint_axis=(1, 0, 0))
        second = self.world.new_link((0, 0, 0), (0, 0, 0), (0, 0, 0), 'prismatic', -6, 6, joint_axis=(0, 1, 0),
                                     parent=first)
        droid = self.world.new_link((0, 0, 0), (0, 0, 0), (0.75, 1, 1), 'revolute', -calc.RAD180, calc.RAD180,
                                    parent=second, mesh_filepath="input-meshes/droids.blend", object_name="droids_3",
                                    new_mesh_name="robot")
        self.start_state.extend((0, 0, 0))
        self.goal_space.extend(((0, 0), (5, 5)))
        self.goal_space_append((calc.RAD90, calc.RAD90))

        self.world.create_goal_duplicate((0, 5, 0), (0, 0, calc.RAD90))

        # add obstacle
        y = random() * 2.75 + 0.75
        first = self.world.new_link((0, y, 0.125), (0, 0, 0), (0, 0, 0), 'prismatic', -6, 6, joint_axis=(1, 0, 0))
        second = self.world.new_link((0, 0, 0), (0, 0, 0), (0, 0, 0), 'prismatic', -6, 6, joint_axis=(0, 1, 0),
                                     parent=first)
        obstacle = self.world.new_link((0, 0, 0), (0, 0, 0), (3.5, 0.35, 0.25), 'revolute', -calc.RAD180, calc.RAD180,
                                    parent=second)
        self.start_state.extend((0, 0, 0))
        self.goal_space.extend(((-16, 16), (-16, 16)))
        self.goal_space_append((-calc.RAD180, calc.RAD180))

        # add door
        self.world.new_door((1.5, 4, 0.5), (0, 0, calc.RAD180), (3.2, 0.2, 1), top_handle=False)
        self.start_state.append(0)
        self.goal_space_append((0, calc.RAD90))

        # add walls
        self.world.new_link((0, -4, 0.125), (0, 0, 0), (3.8, 0.2, 0.25), 'fixed', name="wall_left", material=color.GRAY)
        self.world.new_link((2, 0, 0.125), (0, 0, 0), (0.2, 8, 0.25), 'fixed', name="wall_front", material=color.GRAY)
        self.world.new_link((-2, 0, 0.125), (0, 0, 0), (0.2, 8, 0.25), 'fixed', name="wall_back", material=color.GRAY)

        self.world.export()
        self.world.render_image()
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
        self.world.reset()
        self.world.create_base_link(self.floor_size)

        first = self.world.new_link((0, 0, 0.5), (0, 0, 0), (0, 0, 0), 'prismatic', -16, 16, joint_axis=(1, 0, 0))
        second = self.world.new_link((0, 0, 0), (0, 0, 0), (0, 0, 0), 'prismatic', -16, 16, joint_axis=(0, 1, 0),
                                     parent=first)
        droid = self.world.new_link((0, 0, 0), (0, 0, 0), (0.75, 1, 1), 'revolute', -calc.RAD180, calc.RAD180,
                                    parent=second, mesh_filepath="input-meshes/droids.blend", object_name="droids_3",
                                    new_mesh_name="robot")

        start = (0, 0, 0)
        # start = (random() * 3 - 1.5, random() * 1.4 - 0.7, random() * calc.RAD360 - calc.RAD180)
        self.start_state.extend(start)

        goal = (random() * 3 - 1.5, random() * 1.4 - 0.7 + 3, random() * calc.RAD360 - calc.RAD180)
        self.goal_space_append((goal[0], goal[0]))
        self.goal_space_append((goal[1], goal[1]))
        self.goal_space_append((goal[2], goal[2]))

        self.world.create_goal_duplicate((goal[0], goal[1], 0), (0, 0, goal[2]))

        self.world.new_link((0, -1.4, 0.5), (0, 0, 0), (4, 0.2, 1), 'fixed', name="wall_left", material=color.GRAY)
        self.world.new_link((0, 4.4, 0.5), (0, 0, 0), (4, 0.2, 1), 'fixed', name="wall_right", material=color.GRAY)
        self.world.new_link((0, 1.5, 0.5), (0, 0, 0), (4, 0.2, 1), 'fixed', name="wall_mid", material=color.GRAY)
        self.world.new_link((-2.1, 1.5, 0.5), (0, 0, 0), (0.2, 6, 1), 'fixed', name="wall_back", material=color.GRAY)
        self.world.new_link((2.1, 1.5, 0.5), (0, 0, 0), (0.2, 6, 1), 'prismatic', -3, 3, joint_axis=(0, 1, 0))
        self.goal_space.append((-2, 2))
        self.start_state.append(0)

        self.world.export()
        self.world.render_image()
        return 0
