# Puzzle Generator
Generate synthetic datasets for robot manipulation tasks using procedural content generation (PCG).
The generator engine is written in Python and utilizes Blender's scripting API together with the
[Phobos](https://github.com/dfki-ric/phobos/tree/blender2.9)
add-on.

Generated Puzzles:
* use PCG
* are manipulation challenges
* are generated according to a given config with constraints
* include stochasticity within those constraints
* have _interlocking dependencies_ between joints
* can be used for training or benchmarking an agent/algorithm
* are URDF files but the output can be anything that is supported by Blender/Phobos


Here is an example of a generated grid world puzzle being solved by a fetch robot.
The translucent green box marks the goal position for the green link:

[![Link to YouTube Video](https://img.youtube.com/vi/jRAhU_AycYU/0.jpg)](https://youtu.be/jRAhU_AycYU)


Some of the sampling algorithms need
[PyBullet](https://github.com/bulletphysics/bullet3)
with
[OMPL](https://ompl.kavrakilab.org/core/installation.html)
for evaluation during the sampling process.
The others can be used without the installation of PyBullet and OMPL


## Install (limited)
- Download and install [Blender 2.93](https://www.blender.org/download)
- Install the Phobos add-on for Blender from the correct [branch for version 2.93](https://github.com/dfki-ric/phobos/tree/blender2.9)

Features: create_custom_urdf.py, SimpleSlidersSampler, GridWorldSampler, Lockbox2017Sampler, EscapeRoomSampler, MoveTwiceSampler, MoveNTimesSampler, RoomsSampler


## Install (complete)
The above steps plus:
- Install [OMPL](https://ompl.kavrakilab.org/core/installation.html)
- Install [PyBullet](https://github.com/bulletphysics/bullet3)

Features: pybullet_simulation.py, ContinuousSpaceSampler, LockboxRandomSampler


## Run
There are two possible scripts that you can run. You can either create a custom URDF file or use the puzzle generator to generate a puzzle:


### Create a custom URDF
Adjust the script ```create_custom_urdf.py``` to your requirements and run it with:

```bash
./blender-2.93 --background --python create_custom_urdf.py
```

### Create a puzzle
Adjust the script ```puzzle_generator.py``` to your requirements and run it with:

```bash
./blender-2.93 --background --python puzzle_generator.py
```
