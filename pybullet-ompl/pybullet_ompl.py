import pb_ompl
import sys
import os
import pybullet as pb
from ast import literal_eval

DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)

# default values for optional arguments
# provide arguments when executing this script with python3 like so:
# python3 pybullet_ompl.py /absolute/path/to/urdf/puzzle.urdf "[0, 0]" "[(1, 1), (0, 1)]" etc...
# or change the default values in the following section
FILEPATH_FOR_INPUT = sys.argv[1] if len(sys.argv) > 1 else "/absolute/path/to/urdf/puzzle.urdf"
START_STATE = literal_eval(sys.argv[2]) if len(sys.argv) > 2 else [0, 0]  # 2d example
GOAL_SPACE = literal_eval(sys.argv[3]) if len(sys.argv) > 3 else [(1, 1), (0, 1)]  # 2d example
ALLOWED_PLANNING_TIME = literal_eval(sys.argv[4]) if len(sys.argv) > 4 else 5.
SHOW_GUI = literal_eval(sys.argv[5]) if len(sys.argv) > 5 else True
PLANNER = sys.argv[6] if len(sys.argv) > 6 else "RRTConnect"
HAVE_EXACT_SOLUTION = literal_eval(sys.argv[7]) if len(sys.argv) > 7 else True

if FILEPATH_FOR_INPUT == "/absolute/path/to/urdf/puzzle.urdf":
    print("""\n\tPLEASE provide arguments when executing this script with python3 like so:
        python3 pybullet_ompl.py /absolute/path/to/urdf/puzzle.urdf "[0, 0]" "[(1, 1), (0, 1)]" etc...
        or change the default values in this script (pybullet_ompl.py)\n""")

if SHOW_GUI:
    pb.connect(pb.GUI)
else:
    pb.connect(pb.DIRECT)

# load robot
robot_id = pb.loadURDF(FILEPATH_FOR_INPUT, (0, 0, 0), useFixedBase=1)
robot = pb_ompl.PbOMPLRobot(robot_id)

# setup pb_ompl
pb_ompl_interface = pb_ompl.PbOMPL(robot)
pb_ompl_interface.set_planner(PLANNER)

robot.set_state(START_STATE)
found_solution, path = pb_ompl_interface.plan(GOAL_SPACE, ALLOWED_PLANNING_TIME)

# pb_ompl_interface.is_state_valid()
# pb_ompl_interface.ss.

if HAVE_EXACT_SOLUTION:
    found_solution = pb_ompl_interface.ss.haveExactSolutionPath()
# print("PATH FOUND: " + str(found_solution))
if found_solution and SHOW_GUI:
    pb_ompl_interface.execute(path)

if found_solution:
    sys.exit(0)
else:
    sys.exit(1)
