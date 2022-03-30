from os import path, stat
from subprocess import run
from shutil import copytree


def copy_puzzle(puzzle_directory, robowflex_workspace="../rb_ws"):
    puzzle_directory = path.normpath(puzzle_directory)
    puzzle_name = puzzle_directory.split('/')[-1]
    destination = path.join(robowflex_workspace, "src/robowflex/robowflex_dart/include/io/envs", puzzle_name)
    copytree(puzzle_directory, destination, dirs_exist_ok=True)


def create_srdf(urdf_path, robowflex_workspace="../rb_ws"):
    run(["python3", robowflex_workspace + "/src/robowflex/robowflex_dart/include/io/create_srdf.py", urdf_path])


def adjust_absolute_filepaths(robowflex_workspace="../rb_ws"):
    run(["python3", "filepath_organizer.py"], cwd=robowflex_workspace + "/src/robowflex/robowflex_dart/include/io")


def test_solvability(urdf_path="puzzles/simple_sliders/urdf/simple_sliders.urdf", planning_time=5,
                     robowflex_workspace="../rb_ws", adjust_filepaths=False):
    puzzle_directory = path.dirname(path.dirname(urdf_path))
    puzzle_name = puzzle_directory.split('/')[-1]

    copy_puzzle(puzzle_directory, robowflex_workspace)
    create_srdf(robowflex_workspace + "/src/robowflex/robowflex_dart/include/io/envs/" + puzzle_name + "/urdf/"
                + puzzle_name + ".urdf")

    # TODO: support URDFs with meshes
    if adjust_filepaths:
        adjust_absolute_filepaths(robowflex_workspace)

    # TODO: catch errors in urdf2config.py e.g. no srdf
    run(["./solve_puzzle", puzzle_name, str(planning_time)], cwd=robowflex_workspace + "/devel/lib/robowflex_dart/")

    if stat(robowflex_workspace + "/src/robowflex/robowflex_dart/include/io/path_result/" + puzzle_name +
            ".txt").st_size == 0:
        print("NO SOLUTION FOUND!")
        return 1
    else:
        print("FOUND SOLUTION!")
        return 0
