from subprocess import run


def test_urdf(input_path, start_state, goal_space, allowed_planning_time=5., show_gui=False, planner="RRTConnect",
              have_exact_solution=True, verbose=False):
    """Test solvability with [pybullet_ompl](https://github.com/lyf44/pybullet_ompl) as a subprocess."""
    start_state = str(start_state)
    goal_space = str(goal_space)
    if verbose:
        print("starting subprocess pybullet_ompl to test solvability")
        print("input:", input_path)
        print("start state:", start_state)
        print("goal space:", goal_space)
    result = run(["python3", "pybullet-ompl/pybullet_ompl.py", input_path, start_state, goal_space,
                  str(allowed_planning_time), str(show_gui), planner, str(have_exact_solution)]).returncode
    if verbose:
        print("returned from subprocess")
        if result == 0:
            print("FOUND SOLUTION!")
        else:
            print("DID NOT FIND SOLUTION!")
    return result
