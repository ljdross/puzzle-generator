from subprocess import run


def solve(urdf_path, start_state, goal_space, allowed_planning_time=5., show_gui=False, planner="RRTConnect",
          have_exact_solution=True, verbose=True, only_check_start_state_validity=False):
    """Test solvability with [pybullet_ompl](https://github.com/lyf44/pybullet_ompl) as a subprocess."""
    start_state = str(start_state)
    goal_space = str(goal_space)
    if verbose:
        if only_check_start_state_validity:
            print("starting subprocess pybullet_ompl to check start state validity")
        else:
            print("starting subprocess pybullet_ompl to test solvability")
        print("input:", urdf_path)
        print("start state:", start_state)
        print("goal space:", goal_space)
    result = run(["python3", "pybullet-ompl/pybullet_ompl.py", urdf_path, start_state, goal_space,
                  str(allowed_planning_time), str(show_gui), planner, str(have_exact_solution),
                  str(only_check_start_state_validity)]).returncode
    if verbose:
        print("returned from subprocess")
        if result == 0:
            if have_exact_solution:
                print("FOUND EXACT SOLUTION!")
            else:
                print("FOUND SOLUTION!")
        else:
            if have_exact_solution:
                print("NO EXACT SOLUTION FOUND!")
            else:
                print("NO SOLUTION FOUND!")
    return result
