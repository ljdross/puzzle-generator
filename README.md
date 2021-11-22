# Puzzle Generator
Python script for Blender's scripting API to generate logic geometric puzzles utilizing the "bpy" library, the [Phobos](https://github.com/dfki-ric/phobos/tree/blender2.9) add-on for Blender and [PyBullet](https://github.com/bulletphysics/bullet3) with [OMPL](https://ompl.kavrakilab.org/core/installation.html).


## Install
- Download and install [Blender 2.93](https://www.blender.org/download)
- Install the Phobos add-on for Blender from the correct [branch for version 2.93](https://github.com/dfki-ric/phobos/tree/blender2.9)
- Install [OMPL](https://ompl.kavrakilab.org/core/installation.html) (not required for just creating a custom URDF)
- Install [PyBullet](https://github.com/bulletphysics/bullet3) (not required for just creating a custom URDF)


## Run
There are two possible scripts that you can run. You can either create a custom URDF file or use the puzzle generator to generate a puzzle:


### Create a custom URDF
Adjust the script ```src/create_custom_urdf.py``` to your requirements and run it with:

```bash
./blender-2.93 --background --python src/create_custom_urdf.py
```

### Create a puzzle
Adjust the script ```puzzle_generator.py``` to your requirements and run it with:

```bash
./blender-2.93 --background --python puzzle_generator.py
```
