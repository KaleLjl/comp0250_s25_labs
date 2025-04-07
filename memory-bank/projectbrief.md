# Project Brief: COMP0250 Coursework 1 (CW1)

## Overview

This project focuses on the development of solutions for the first robotics coursework assignment (CW1) for the COMP0250: Robot Sensing, Manipulation and Interaction module. The assignment involves programming a simulated Panda robot arm in the Gazebo environment to perform perception and manipulation tasks, specifically pick-and-place, object detection, and color identification. Coursework 2 (CW2) is currently out of scope.

## Core Objectives

The primary goal is to implement robust ROS-based systems capable of:

1.  **Sensing & Perception:** Utilizing RGB-D camera data and the Point Cloud Library (PCL) for:
    *   Detecting and localizing objects (boxes, baskets).
    *   Identifying object properties (color).
2.  **Manipulation & Planning:** Employing MoveIt! for:
    *   Planning and executing collision-free pick-and-place trajectories.
    *   Grasping different object types effectively.
    *   Operating within task-specific constraints and environments, including obstacle avoidance.
3.  **System Integration:** Building complete ROS packages that integrate perception, planning, and execution components to solve multi-step tasks defined by ROS services.

## Coursework Specifics

### Coursework 1 (CW1 - 40% Grade)

*   **Focus:** Introduction to pick-and-place, object detection, and color identification.
*   **Tasks:**
    1.  Pick a known cube (given pose) and place it in a known basket (given pose).
    2.  Identify the colors (red, blue, purple) of baskets at specified locations using camera data.
    3.  Detect colored boxes and baskets, then sort each box into the basket of the matching color.
*   **Objects:** Simple cubes and baskets.

## Key Technologies

*   **Operating System:** ROS (Robot Operating System)
*   **Simulation:** Gazebo
*   **Motion Planning:** MoveIt!
*   **Perception:** PCL (Point Cloud Library), OpenCV (cv2)
*   **Languages:** C++ (recommended), Python

## Deliverables

*   A single ROS package: `cw1_team_<team_number>`.
*   The package must contain all source code, launch files (`run_solution.launch`), and configuration required to run the CW1 solution.
*   A comprehensive `README.md` file within the package detailing:
    *   Authors and contributions (time/percentage per task).
    *   License information.
    *   Build and execution instructions.
    *   Any specific notes about the implementation.

## Evaluation Criteria

*   **Correctness:** Successful completion of CW1 tasks according to specifications. Robustness against specified variations. Collision avoidance.
*   **Performance:** Efficiency of algorithms and execution time.
*   **Code Quality:** Clarity, structure, modularity, comments, adherence to ROS best practices.
*   **Documentation:** Completeness and clarity of the README file.
