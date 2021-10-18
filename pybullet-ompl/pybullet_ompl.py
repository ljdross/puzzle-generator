import pb_ompl
import sys
import os
import pybullet as pb
DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(DIR)


FILEPATH_FOR_INPUT = sys.argv[1]
START_STATE = [0, 0]
GOAL_STATE = [1, 1]
SHOW_GUI = True
PLANNER = "RRTConnect"
ALLOWED_PLANNING_TIME = 5.


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

