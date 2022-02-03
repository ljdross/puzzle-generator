from random import seed, random, choice
from subprocess import run
import os
import sys
DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from world import World
import calc


class PuzzleSampler:
    def __init__(self, config, world: World):
        self.world = world
        self.name = config["puzzle_name"]
        self.directory = config["dir_for_output"] + "/" + config["puzzle_name"]
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
        # self.floor_size = config["floor_size"]
        # self.export_entity_srdf = config["export_entity_srdf"]
        # self.export_mesh_dae = config["export_mesh_dae"]
        # self.base_object = None
        # self.movable_objects = []

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

    def make_lower_and_upper_limit(self, distance):
        if distance < 0:
            return distance, 0
        else:
            return 0, distance

    def test_with_pybullet_ompl(self, allowed_planning_time=5., show_gui=False, have_exact_solution=True,
                                planner="RRTConnect", verbose=False):
        """Test solvability with [pybullet_ompl](https://github.com/lyf44/pybullet_ompl) as a subprocess."""
        input_path = self.directory + "/urdf/" + self.name + ".urdf"
        start_state = str(self.start_state)
        print("self.start_state = " + start_state)
        goal_state = str(self.goal_space)
        print("self.goal_space = " + goal_state)
        result = run(["python3", "pybullet-ompl/pybullet_ompl.py", input_path, start_state, goal_state,
                      str(show_gui), str(allowed_planning_time), str(have_exact_solution), planner]).returncode
        if verbose:
            if result == 0:
                print("FOUND SOLUTION!")
            else:
                print("DID NOT FIND SOLUTION!")
        return result


class SimpleSlidersSampler(PuzzleSampler):
    def __init__(self, config, world: World):
        super().__init__(config, world)

    def _create_simple_sliders_puzzle(self):
        """Create a very simple model that only works with prismatic joints (number_revolute_joints must equal 0)."""
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
        self.world.create_base_link()
        self._create_simple_sliders_puzzle()
        self.world.create_collision()
        self.world.export()
        return 0


class ContinuousSpaceSampler(PuzzleSampler):
    def __init__(self, config, world: World):
        super().__init__(config, world)
        self.attempts = 50
        self.planning_time = 0.1
        self.next_joint_time_multiplier = 2
        self.first_test_time_multiplier = 1.5
        self.area_size = 3

    def _get_random_limit_distance(self, is_prismatic):
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
            limit_distance = self._get_random_limit_distance(True)
            self.world.new_object((self.start_point[0], self.start_point[1], 0.5), (-calc.RAD90, 0, rotation), (1, 1, self.prismatic_length),
                            'prismatic', lower_limit=0, upper_limit=limit_distance, add_to_goal_space=False)
            self.prismatic_joints_target -= 1
            self.start_point = self._calculate_next_start_point(True, self.start_point, rotation, limit_distance)
        else:
            # create revolute joint
            limit_distance = self._get_random_limit_distance(False)
            if limit_distance > 0:
                self.world.new_object((self.start_point[0], self.start_point[1], 0.5), (0, 0, rotation), (self.revolute_length, 1, 1),
                                'revolute', lower_limit=0, upper_limit=limit_distance, add_to_goal_space=False)
            else:
                self.world.new_object((self.start_point[0], self.start_point[1], 0.5), (0, 0, rotation), (self.revolute_length, 1, 1),
                                'revolute', lower_limit=limit_distance, upper_limit=0, add_to_goal_space=False)
            self.revolute_joints_target -= 1
            self.start_point = self._calculate_next_start_point(False, self.start_point, rotation, limit_distance)
        self.goal_space_append((limit_distance, limit_distance))

    def _sample_joint(self):
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
                self.world.new_object((new_point[0], new_point[1], 0.5), (-calc.RAD90, 0, rotation), (1, 1, self.prismatic_length),
                                'prismatic', lower_limit=0, upper_limit=0, add_to_goal_space=False)
                is_prismatic = True
            else:
                # create immovable revolute joint (joint limits = 0)
                self.world.new_object((new_point[0], new_point[1], 0.5), (0, 0, rotation), (self.revolute_length, 1, 1),
                                'revolute', lower_limit=0, upper_limit=0, add_to_goal_space=False)
                is_prismatic = False
            self.world.create_collision(self.world.movable_objects[-1])
            self.world.export()
            result = self.test_with_pybullet_ompl(self.planning_time * self.first_test_time_multiplier)
            if result == 0:
                # can be solved with the immovable joint
                # we do not want that
                # this new joint should block the previous joint
                self.world.remove_last_object()
                continue
            else:
                # the new (immovable) joint successfully blocks the previously solvable puzzle
                # now make it movable
                limit_distance = self._get_random_limit_distance(is_prismatic)
                self.world.set_limit_of_active_object_and_add_to_goal_space(limit_distance, is_prismatic)
                limits_tuple = self.make_lower_and_upper_limit(limit_distance)
                self.goal_space_append(limits_tuple)
                self.start_state.append(0)

                # and check solvability again
                result = self.test_with_pybullet_ompl(self.planning_time)
                if result == 0:
                    self.start_point = self._calculate_next_start_point(is_prismatic, new_point, rotation, limit_distance)
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

    def _create_sampleworld_puzzle(self):
        """
        Sample links with joints iteratively
        """
        self._sample_first_joint()
        self.world.create_collision()
        for i in range(1, self.total_number_joints):
            result = self._sample_joint()
            if result != 0:
                print("\U000026D4 " * 64)
                print("Could NOT sample link" + str(i), "after", self.attempts, "attempts!")
                print("\U000026D4 " * 64)
                return result
            print("Successfully sampled link" + str(i), "\U000026F3 " * i)
            self.planning_time *= self.next_joint_time_multiplier
        print("\U000026F3 " * 32)
        print("SUCCESS! Sampled", self.total_number_joints, "links!")
        print("\U000026F3 " * 32)
        return 0

    def build(self):
        """Build complete model in Blender and export to URDF. Sample random positions for joints."""
        self.world.reset()
        self.world.create_base_link()
        result = self._create_sampleworld_puzzle()
        if result == 0:
            return 0
        else:
            return result
