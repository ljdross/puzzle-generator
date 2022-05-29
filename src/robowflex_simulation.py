from os import path, stat
from subprocess import run
from shutil import copytree

# install https://github.com/servetb/robowflex and specify its location:
ROBOWFLEX_WORKSPACE = "../../rb_ws"

# install https://github.com/aorthey/ompl_benchmark_plotter and specify its location:
OMPL_BENCHMARK_PLOTTER = "../ompl_benchmark_plotter"


def copy_puzzle(puzzle_directory):
    puzzle_directory = path.normpath(puzzle_directory)
    puzzle_name = puzzle_directory.split('/')[-1]
    destination = path.join(ROBOWFLEX_WORKSPACE, "src/robowflex/robowflex_dart/include/io/envs", puzzle_name)
    copytree(puzzle_directory, destination, dirs_exist_ok=True)


def create_srdf(urdf_path):
    run(["python3", ROBOWFLEX_WORKSPACE + "/src/robowflex/robowflex_dart/include/io/create_srdf.py", urdf_path])


def adjust_absolute_filepaths():
    run(["python3", "filepath_organizer.py"], cwd=ROBOWFLEX_WORKSPACE + "/src/robowflex/robowflex_dart/include/io")


def solve(urdf_path="puzzles/simple_sliders/urdf/simple_sliders.urdf", planning_time=5, adjust_filepaths=False,
          benchmark_runs=0, animation=False):
    puzzle_directory = path.dirname(path.dirname(urdf_path))
    puzzle_name = puzzle_directory.split('/')[-1]

    copy_puzzle(puzzle_directory)
    create_srdf(ROBOWFLEX_WORKSPACE + "/src/robowflex/robowflex_dart/include/io/envs/" + puzzle_name + "/urdf/"
                + puzzle_name + ".urdf")

    if adjust_filepaths:
        adjust_absolute_filepaths()

    if not benchmark_runs:
        path_res = ROBOWFLEX_WORKSPACE + "/src/robowflex/robowflex_dart/include/io/path_result/" + puzzle_name + ".txt"
        with open(path_res, "w"):  # reset result file
            pass

        if animation:
            run(["./solve_puzzle_animation", puzzle_name, str(planning_time)],
                cwd=ROBOWFLEX_WORKSPACE + "/devel/lib/robowflex_dart/")
        else:
            run(["./solve_puzzle", puzzle_name, str(planning_time)],
                cwd=ROBOWFLEX_WORKSPACE + "/devel/lib/robowflex_dart/")

        if stat(path_res).st_size == 0:  # size of result file
            print("NO SOLUTION FOUND!")
            return 1
        else:
            print("FOUND SOLUTION!")
            return 0

    else:
        run(["./benchmark_main", puzzle_name, str(planning_time), str(benchmark_runs)],
            cwd=ROBOWFLEX_WORKSPACE + "/devel/lib/robowflex_dart/")
        run(["python3", OMPL_BENCHMARK_PLOTTER + "/ompl_benchmark_plotter.py",
             ROBOWFLEX_WORKSPACE + "/src/robowflex/robowflex_dart/include/io/db_files/" + puzzle_name + ".db",
             "--min-time", "0.04", "--max-time", str(planning_time)])
        print("benchmark results:")
        print("file://" + path.abspath(ROBOWFLEX_WORKSPACE + "/src/robowflex/robowflex_dart/include/io/db_files"))
        print("file://" + path.abspath(ROBOWFLEX_WORKSPACE + "/src/robowflex/robowflex_dart/include/io/db_files/"
                                       + puzzle_name + ".db"))
