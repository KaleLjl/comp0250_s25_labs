# Product Context: COMP0250 Coursework 1 (CW1)

## Problem Domain

This project addresses the challenges presented in the COMP0250: Robot Sensing, Manipulation and Interaction coursework 1 (CW1). The core problem is to develop an autonomous robotic system within a simulated Gazebo environment that can perceive its surroundings using RGB-D sensors and manipulate objects (cubes, baskets) using a Panda robot arm. Coursework 2 is currently out of scope.

## Project Purpose

The primary purpose of this project is to successfully complete the requirements of CW1, demonstrating proficiency in key robotics concepts:

1.  **Sensor Integration:** Effectively utilizing RGB-D camera data (point clouds, images) for environmental understanding.
2.  **Object Perception:** Implementing algorithms to detect, locate, and identify objects (cubes, baskets) based on properties like color.
3.  **Motion Planning:** Using MoveIt! to generate safe and effective robot trajectories for manipulation tasks.
4.  **Task Execution:** Integrating perception and manipulation capabilities to perform complex pick-and-place operations according to specific task goals defined via ROS services.
5.  **System Building:** Constructing a well-structured and robust ROS package (`cw1_team_<team_number>`) that solves the defined problems for CW1.

## Functional Goals

The system must be able to:

*   **CW1:**
    *   Pick a known cube and place it in a designated basket.
    *   Visually inspect specified locations and report the color of any baskets found (red, blue, purple) or indicate if the location is empty.
    *   Detect colored boxes and baskets, and sort the boxes into their corresponding colored baskets.

## User Experience (Developer/Evaluator)

*   **Setup:** The solution should be easily buildable and runnable using standard `catkin build` and `roslaunch` commands within the provided `comp0250_s25_labs` environment.
*   **Execution:** Launching `run_solution.launch` should start all necessary nodes for CW1. Tasks are initiated via `rosservice call /task X` (where X is 1, 2, or 3 for CW1).
*   **Debugging:** Code should be clear and well-structured to facilitate understanding and debugging.
*   **Evaluation:** The system's behavior (robot motion, task completion) and outputs (service responses, console logs if any) should clearly demonstrate correct task execution according to the evaluation criteria.
