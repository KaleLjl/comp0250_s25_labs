# Technical Context: COMP0250 Coursework 1 (CW1)

## Core Technologies

*   **Robot Operating System (ROS):** The fundamental framework for communication and package management. Specific version likely tied to the `comp0250_s25_labs` environment (e.g., ROS Noetic).
*   **Gazebo:** The simulation environment where the robot and tasks operate.
*   **MoveIt!:** ROS framework for motion planning, manipulation, kinematics, and collision checking for the Panda robot arm.
*   **Point Cloud Library (PCL):** Used for processing 3D point cloud data from the simulated RGB-D sensor for object detection (cubes, baskets) and localization.
*   **OpenCV (cv2):** Likely used for image processing tasks, potentially complementing PCL for color identification or 2D feature analysis.

## Programming Languages

*   **C++:** Recommended by the coursework description, especially for efficient integration with MoveIt! and PCL.
*   **Python:** Permitted, but potentially less performant for intensive PCL/MoveIt! operations. Used in the provided `world_spawner` scripts.

## Development Environment & Setup

*   **Base Repository:** `comp0250_s25_labs` (requires cloning with `--recurse-submodules`).
*   **Build System:** `catkin` (`catkin build`).
*   **Workspace Setup:** Standard ROS workspace structure (`src/`, `build/`, `devel/`). Requires sourcing `devel/setup.bash`.
*   **Provided Packages:**
    *   `cw1_world_spawner`: Sets up CW1 tasks via ROS services and spawns objects/environments in Gazebo. Contains service definitions (`Task1Service.srv`, `Task2Service.srv`, `Task3Service.srv`). *Note: Modifications allowed for testing but not submitted.*
    *   `cw1_team_x`: Template package for the CW1 solution. Must be renamed to `cw1_team_<team_number>`.
    *   Other dependencies within `comp0250_s25_labs` (e.g., `panda_description`, `panda_moveit_config`, `realsense_gazebo_plugin`).
*   **Execution:** The CW1 solution is launched via `roslaunch cw1_team_<team_number> run_solution.launch`. Task initiation via `rosservice call /task X` (X=1, 2, or 3).

## Key Interfaces & Data

*   **RGB-D Sensor Topics:** Camera data (e.g., `/r200/depth/image_raw`, `/r200/color/image_raw`, `/r200/depth/points`) are the *only* permitted source for object perception. Direct access to Gazebo model states is forbidden.
*   **ROS Services:**
    *   `/task1_start`, `/task2_start`, `/task3_start`: Services provided by the `cw1_team_<team_number>` node(s) to handle task requests initiated by `cw1_world_spawner`. Defined in `cw1_world_spawner/srv/`.
    *   `/task 1`, `/task 2`, `/task 3`: Services called by the user/evaluator to trigger `cw1_world_spawner` to set up and initiate a specific CW1 task.
*   **Data Types:** Standard ROS messages relevant to CW1 (e.g., `geometry_msgs/PoseStamped`, `geometry_msgs/PointStamped`, `sensor_msgs/PointCloud2`, `std_msgs/String`).

## Constraints & Restrictions

*   **Submission:** Only the `cw1_team_<team_number>` directory is submitted.
*   **Launch File:** All necessary nodes for the CW1 solution must be launched from the single `run_solution.launch` file within the `cw1_team_<team_number>` package.
*   **Dependencies:** Allowed to add dependencies on standard ROS libraries (e.g., `sensor_msgs`, `geometry_msgs`, `moveit`, `cv_bridge`, `pcl_ros`). Non-standard dependencies require clarification.
*   **Robot Parameters:** Tweaking internal robot parameters (speed, acceleration) is forbidden.
*   **Perception Source:** Must use RGB-D sensor topics; cannot query Gazebo directly for object locations/poses.
*   **Team Contribution:** Roughly equal contribution expected from team members, documented in the README.
