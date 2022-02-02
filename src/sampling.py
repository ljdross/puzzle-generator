from random import seed, random, choice
from subprocess import run
import os
import sys
DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)
from world import World
import calc


class Sampler:
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
        self.floor_size = config["floor_size"]
        self.export_entity_srdf = config["export_entity_srdf"]
        self.export_mesh_dae = config["export_mesh_dae"]
        self.base_object = None
        self.movable_objects = []

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
        self.goal_space.append((lower_limit, upper_limit))

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


class SimpleSliders(Sampler):
    def __init__(self, config, world: World):
        super().__init__(config, world)

    def create_simple_sliders_puzzle(self):
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
        self.start_state = [0] * self.number_prismatic_joints
        self.world.reset()
        self.world.create_base_link()
        self.create_simple_sliders_puzzle()
        self.world.create_collision()
        self.world.export()
        return 0
