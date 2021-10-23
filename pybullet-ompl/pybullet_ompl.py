import pb_ompl
import sys
import os
import pybullet as pb
from ast import literal_eval
DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)

# default values for optional args
FILEPATH_FOR_INPUT = sys.argv[1] if len(sys.argv) > 1 else "/home/userone/ba/puzzle-generator/puzzles/simple_sliders/urdf/simple_sliders.urdf"
START_STATE = literal_eval(sys.argv[2]) if len(sys.argv) > 2 else [0, 0]
GOAL_STATE = literal_eval(sys.argv[3]) if len(sys.argv) > 3 else [1, 1]
SHOW_GUI = literal_eval(sys.argv[4]) if len(sys.argv) > 4 else True
ALLOWED_PLANNING_TIME = literal_eval(sys.argv[5]) if len(sys.argv) > 5 else 5.
PLANNER = sys.argv[6] if len(sys.argv) > 6 else "RRTConnect"

if SHOW_GUI:
    pb.connect(pb.GUI)
else:
    pb.connect(pb.DIRECT)

pb.setTimeStep(1./60.)

# load robot
robot_id = pb.loadURDF(FILEPATH_FOR_INPUT, (0, 0, 0), useFixedBase=1)
robot = pb_ompl.PbOMPLRobot(robot_id)

# setup pb_ompl
pb_ompl_interface = pb_ompl.PbOMPL(robot)
pb_ompl_interface.set_planner(PLANNER)

robot.set_state(START_STATE)
found_solution, path = pb_ompl_interface.plan(GOAL_STATE, ALLOWED_PLANNING_TIME)
# print("PATH FOUND: " + str(found_solution))
if found_solution and SHOW_GUI:
    pb_ompl_interface.execute(path)

if found_solution:
    sys.exit(0)
else:
    sys.exit(1)

