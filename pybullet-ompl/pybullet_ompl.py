import pb_ompl
import sys
import os
import pybullet as p
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
ONLY_CHECK_START_STATE_VALIDITY = literal_eval(sys.argv[8]) if len(sys.argv) > 8 else False
URDF_USE_SELF_COLLISION = literal_eval(sys.argv[9]) if len(sys.argv) > 9 else False  # seems to have no effect

if FILEPATH_FOR_INPUT == "/absolute/path/to/urdf/puzzle.urdf":
    print("""\n\tPLEASE provide arguments when executing this script with python3 like so:
        python3 pybullet_ompl.py /absolute/path/to/urdf/puzzle.urdf "[0, 0]" "[(1, 1), (0, 1)]" etc...
        or change the default values in this script (pybullet_ompl.py)\n""")

if SHOW_GUI:
    p.connect(p.GUI)
else:
    p.connect(p.DIRECT)

# load robot
if URDF_USE_SELF_COLLISION:
    robot_id = p.loadURDF(FILEPATH_FOR_INPUT, (0, 0, 0), useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
else:
    robot_id = p.loadURDF(FILEPATH_FOR_INPUT, (0, 0, 0), useFixedBase=1)

robot = pb_ompl.PbOMPLRobot(robot_id)

# setup pb_ompl
pb_ompl_interface = pb_ompl.PbOMPL(robot)

if ONLY_CHECK_START_STATE_VALIDITY:
    valid = pb_ompl_interface.is_state_valid(START_STATE)
    # print("IS STATE VALID?", valid)
    if valid:
        sys.exit(0)
    else:
        sys.exit(1)

pb_ompl_interface.set_planner(PLANNER)

robot.set_state(START_STATE)
found_solution, path = pb_ompl_interface.plan(GOAL_SPACE, ALLOWED_PLANNING_TIME)

if HAVE_EXACT_SOLUTION:
    found_solution = pb_ompl_interface.ss.haveExactSolutionPath()
# print("PATH FOUND:", path)
if found_solution and SHOW_GUI:
    pb_ompl_interface.execute(path)

if found_solution:
    sys.exit(0)
else:
    sys.exit(1)
