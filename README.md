# COMP0250: Robot Sensing, Manipulation and Interaction - Coursework 2 Instructions

This document provides detailed instructions for setting up and running Coursework 2 for COMP0250.

## Repository Setup

If you haven't cloned the repository yet:
```bash
git clone https://github.com/surgical-vision/comp0250_s25_labs.git --recurse-submodules
```

If you already have the repository, update it:
```bash
cd comp0250_s25_labs
git pull --recurse-submodules
```

## Package Setup

The coursework requires setting up a team-specific package:

1. Create your team package by copying the template:
   ```bash
   cp -r src/cw2_team_x src/cw2_team_<your_team_number>
   ```

2. Update the following files with your team number:
   - The package folder name: `src/cw2_team_<your_team_number>`
   - In `launch/run_solution.launch` line 17: Change the package name
   - In `package.xml` line 3: Update the package name
   - In `CMakeLists.txt` line 2: Update the project name

## Building the Project

1. Navigate to the project directory:
   ```bash
   cd ~/comp0250_s25_labs
   ```

2. Build the project using catkin:
   ```bash
   catkin build
   ```

3. Source the setup file:
   ```bash
   source devel/setup.bash
   ```

## Running the Solution

Launch your team's solution:
```bash
roslaunch cw2_team_<your_team_number> run_solution.launch
roslaunch cw2_team_21 run_solution.launch
```

This will start:
- The Panda robot model in Gazebo
- The world spawner for the coursework
- The octomap server for environment mapping
- Your team's solution node

## Running the Tasks

After launching your solution, you can run each task using ROS service calls:

### Task 1: Pick and Place at Given Positions
```bash
rosservice call /task 1
```
- Robot picks up a shape (nought or cross) from a given position
- Places it into a brown basket
- Shape can have any orientation
- Size is 40mm

### Task 2: Shape Detection
```bash
rosservice call /task 2
```
- Three shapes are spawned: two reference shapes and one mystery shape
- Robot must determine which reference shape the mystery shape matches
- Returns 1 or 2 to indicate which reference shape matches

### Task 3: Planning and Execution
```bash
rosservice call /task 3
```
- Multiple shapes (up to 7) and obstacles (up to 4) are spawned
- Robot must:
  1. Count the total number of shapes (excluding obstacles)
  2. Determine which shape type (nought or cross) is more common
  3. Pick and place one example of the most common shape into the basket
- Shapes can vary in size (20mm, 30mm, or 40mm)

## Debugging

If you encounter issues:

1. Check ROS console output for error messages
2. Verify your package is properly built and sourced
3. Ensure all dependencies are installed
4. Examine your solution's code for logical errors

## Important Parameters

The world spawner allows modification of parameters for testing:

- Task 1: `T1_ANY_ORIENTATION` (True/False)
- Task 2: `T2_ANY_ORIENTATION` (True/False), `T2_GROUND_PLANE_NOISE` (0e-3/50e-3)
- Task 3: `T3_ANY_ORIENTATION` (True/False), `T3_N_OBSTACLES` (0/2/3/4), `T3_USE_MULTIPLE_SIZES` (True/False)

You can modify these in `src/cw2_world_spawner/scripts/world_spawner.py` to make testing easier during development.
