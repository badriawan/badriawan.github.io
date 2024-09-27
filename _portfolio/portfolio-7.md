---
title: "SLAM and Obstacle Avoidance Implementation on The Construct Platform"
excerpt: Student Project <br/><img src='/images/nav2.gif'>"
collection: portfolio
---


## Build a SLAM and Obstacle Avoidance Implementation Using Nav2 System

<p align="center">
<img src="/images/nav2.gif?raw=true" center=true width="55%"/>
</p>

## Brief Review

- The first part of this project involves implementing SLAM (Simultaneous Localization and Mapping) for the TurtleBot3 robot using the Nav2 stack in ROS2.

- The final part focuses on adding obstacle avoidance capabilities, enabling the robot to navigate autonomously in environments with dynamic obstacles.

## Project Tree

- **config**
    - `nav2_params.yaml`
- **include**
    - `slam_lib.hpp`
    - `nav_node.hpp`
- **launch**
    - `slam_navigation.launch.py`
- **src**
    - `slam_lib.cpp`
    - `nav_node.cpp`

## Solutions

- **How I Built the System:**

    1. **Set Up the Workspace and ROS2 Packages:** I started by creating a new ROS2 workspace and initializing the necessary package structure, including the `launch`, `config`, `include`, and `src` directories.

    2. **Install and Add Necessary Libraries:** I installed required ROS2 packages and dependencies such as `nav2_bringup`, `nav2_common`, `nav_msgs`, `tf2_msgs`, `rclcpp`, and `Eigen`. I also ensured that the TurtleBot3 packages were properly installed and configured.

    3. **Create Header Files (`.hpp`):** I developed two header files:

        - `slam_lib.hpp`: Contains declarations for SLAM-related functions and data structures.
        - `nav_node.hpp`: Defines the navigation node's class variables and methods, including path planning and obstacle avoidance functionalities.

    4. **Implement Source Files (`.cpp`):** I wrote the source files:

        - `slam_lib.cpp`: Implements the SLAM algorithms using Nav2's integration with `slam_toolbox`.
        - `nav_node.cpp`: Serves as the ROS2 node responsible for handling navigation tasks, such as path planning, control, and obstacle avoidance.

    5. **Create YAML Configuration Files:** I created `nav2_params.yaml` in the `config` directory to specify parameters for the Nav2 stack, including settings for the SLAM algorithm, costmaps, planners, and controllers.

    6. **Create Launch Files:** I developed `slam_navigation.launch.py` in the `launch` directory to start all the necessary nodes, bringing up the Nav2 stack alongside the TurtleBot3 simulation in Gazebo.

    7. **Modify `CMakeLists.txt` and `package.xml`:** I updated these files to include the necessary dependencies, libraries, and executables, ensuring that all components are correctly built and linked.

    8. **Compile and Run the Nodes:** I compiled the package using `colcon build` and sourced the setup script. Then, I launched the nodes using the launch file to test the implementation in a simulated environment.

- **How I Performed the SLAM and Obstacle Avoidance Implementation:**

    1. **SLAM Implementation:**

        - Utilized the `slam_toolbox` package integrated with Nav2 to perform real-time SLAM.
        - Configured SLAM parameters in `nav2_params.yaml`, enabling the robot to build a map of the environment while localizing itself within that map.
        - Adjusted parameters such as map update frequency, resolution, and scan matching algorithms to optimize mapping performance.

    2. **Obstacle Avoidance Implementation:**

        - Configured the global and local costmaps in `nav2_params.yaml` to include obstacle detection and avoidance.
        - Set up obstacle layers using sensor data from the robot's LiDAR to detect both static and dynamic obstacles.
        - Adjusted inflation radius and obstacle range to ensure safe distances are maintained from obstacles.
        - Tuned the local planner (e.g., `DWB` or `TebLocalPlanner`) parameters to improve the robot's responsiveness to obstacles.

    3. **Testing and Tuning:**

        - Tested the implementation in Gazebo with various simulated environments containing static and dynamic obstacles.
        - Observed the robot's navigation behavior using RViz2, monitoring topics like `/map`, `/scan`, and `/cmd_vel`.
        - Iteratively adjusted parameters in `nav2_params.yaml` to improve navigation performance, such as:

            - **Global Planner Parameters:** Influencing the overall path planning strategy.
            - **Local Planner Parameters:** Affecting obstacle avoidance and path smoothing.
            - **Recovery Behaviors:** Configuring actions the robot takes when it encounters difficulties, like clearing costmaps or performing a retreat.

## Project Summary

This project demonstrates the implementation of SLAM and obstacle avoidance for a TurtleBot3 robot using the Nav2 navigation stack in ROS2. By integrating `slam_toolbox` for mapping and configuring Nav2 for autonomous navigation, the robot can explore unknown environments while safely avoiding obstacles.

**Key Components:**

- **SLAM (`slam_toolbox`):** Enables the robot to create a map of its environment in real-time and localize itself within that map.

- **Nav2 Stack:**

    - **Global Costmap and Planner:** Used for generating high-level paths from the robot's current position to the goal, taking into account the known static obstacles.
    - **Local Costmap and Controller:** Handles dynamic obstacles and fine-grained control, ensuring the robot can react to changes in the environment in real-time.
    - **Obstacle Avoidance:** Utilizes sensor data to detect obstacles and adjust the robot's path accordingly.

- **Parameter Tuning:** Essential for optimizing performance, including:

    - **Costmap Configuration:** Adjusting parameters like resolution, update frequency, and inflation radius.
    - **Planner and Controller Settings:** Tuning parameters that affect path smoothing, obstacle clearance, and robot dynamics.

By carefully configuring and tuning these components, the robot achieves reliable autonomous navigation with effective obstacle avoidance in various environments.

## Using the Package

Follow these steps to replicate the outcome:

- **Create the Workspace:**

    ```bash
    cd ~
    mkdir -p nav2_ws/src
    cd nav2_ws/src
    ```

- **Clone the Repository:**

    ```bash
    git clone https://github.com/your_username/your_nav2_package.git
    ```

- **Install Dependencies:**

    - Ensure that you have installed all necessary dependencies, including TurtleBot3 and Nav2 packages:

        ```bash
        sudo apt update
        sudo apt install ros-foxy-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
        ```

    - Install any additional dependencies specified in the `package.xml` file.

- **Compile and Source the Repository:**

    ```bash
    cd ~/nav2_ws
    colcon build --packages-select your_nav2_package
    source install/setup.bash
    ```

- **Run the Simulation and Navigation Stack:**

    ```bash
    # Launch the Gazebo simulation and Nav2 stack
    ros2 launch your_nav2_package slam_navigation.launch.py
    ```

- **Visualize in RViz2:**

    - Open RViz2 to visualize the robot, map, costmaps, and planned paths:

        ```bash
        rviz2
        ```

    - Load the appropriate RViz2 configuration file if provided.

- **Send Navigation Goals:**

    - Use RViz2's "Nav2 Goal" tool to send navigation goals by clicking on the map.
    - Alternatively, use the command line to send goals:

        ```bash
        ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
        ```

- **Monitor and Adjust Parameters:**

    - Observe the robot's behavior and adjust parameters in `nav2_params.yaml` as needed.
    - Rebuild and relaunch the system after making changes.

## Conclusion

By implementing SLAM and obstacle avoidance using the Nav2 system, the TurtleBot3 robot can autonomously navigate complex environments. This project showcases how integrating mapping, planning, and control components in ROS2 enables advanced robotic behaviors suitable for real-world applications.