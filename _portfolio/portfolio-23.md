---
title: "LQR Controller Implementation for Mobile Robotics Behaviour"
excerpt: Robotisim Project <br/><img src='/images/LQR.png'>"
collection: portfolio
---


## Build a LQR algorithm for enhance robot motion performance
<!-- <p align="center">
<img src = "doc/manipulator.gif?raw=true" center=true width="55%"/>
</p> -->


<details open>
<summary> <b>Brief Review<b></summary>

- First part of the project is creating custom LQR controllers for controlling the mobile robot.

- Last part is implementation of LQR robots using different penalties value in Q and R matriks and analyze the robot behaviour.


<details open>
<summary> <b>Project Tree<b></summary>

- include
    - lqr_lib.hpp
    - lqr_node.hpp
- launch
    - lqr_multi_goals.launch.py
- src
    - lqr_lib.cpp
    - lqr_node.cpp



- **Solutions**:

How I Built the System:

    1. Set Up the Workspace and ROS2 Packages: I started by creating the workspace and ROS2 packages, including the necessary folders such as launch, include, and src.

    2. Install and Add Necessary Libraries: Next, I installed and added the required libraries for the system, such as geometry_msgs, nav_msgs, tf2_msgs, rclcpp, and Eigen. I also created custom libraries as needed.

    3. Create Header Files (.hpp): I developed two header files: lqr_lib.hpp and lqr_node.hpp. The lqr_lib.hpp file contains declarations for methods related to defining the matrices QQ, RR, and KK. The lqr_node.hpp file includes definitions for the state, input, and class variables.

    4. Implement Source Files (.cpp): I wrote the source files lqr_lib.cpp and lqr_node.cpp. The lqr_lib.cpp file computes the LQR controller, while lqr_node.cpp serves as the ROS2 node that processes sensor input data and generates action velocities using the LQR controller.

    5. Create Launch Files: I created launch files that include both the TurtleBot3 empty world node and the LQR node, facilitating simulation and testing.

    6. Modify the CMakeLists.txt File: I updated the CMakeLists.txt file with the correct names of libraries, executable files, and dependencies to ensure proper compilation.

    7. Compile and Run the Nodes: Finally, I compiled the system and ran the nodes to test the implementation.

How I Performed the LQR Control Analysis:

    1. Adjust Q and R Matrix Penalties: I modified the values of the Q and R matrices to analyze how different penalty values affect the robot's behavior.





<details open>
<summary> <b>Project Summary<b></summary>

In this project, we implemented Linear Quadratic Regulator (LQR) algorithms to control a mobile robot navigating through four different waypoints. LQR is an optimal control strategy that determines the control inputs necessary to drive a system's state to a desired target while minimizing a cost function. This cost function typically balances the trade-off between minimizing state errors and minimizing control efforts.

Initially, I set the weighting matrices QQ and RR to default values of 0.8 for the first experiment. In the context of LQR, the matrix QQ penalizes deviations from the desired state, whereas the matrix RR penalizes the use of control inputs. By starting with these equal values, we established a baseline to observe how the system behaves when state errors and control efforts are equally weighted in the cost function.

Subsequently, I adjusted the values of QQ and RR incrementally—both increasing and decreasing them—to study their impact on the system's performance. A higher value of QQ places greater emphasis on minimizing state errors. This means the controller becomes more aggressive in correcting deviations, leading to improved accuracy in reaching the desired target states. However, this can result in higher control efforts, as the system prioritizes precision over energy consumption.

Conversely, increasing the value of RR gives more weight to minimizing the control inputs. This adjustment encourages the controller to use less energy by reducing the magnitude of the control actions. While this conserves energy and reduces actuator wear, it may allow for larger state errors, as the system becomes less aggressive in correcting deviations from the desired trajectory. By carefully tuning QQ and RR, we can achieve an optimal balance between accuracy and energy efficiency tailored to the specific requirements of the mobile robot's task.



<p align="center"> </p>
</details>

<details open>
<summary> <b>Using The Package <b></summary>

- Follow the next steps to replicate the outcome...

- Create the workspace
```sh
    cd ~
    mkdir -p your_workspace/src
    cd your_workspace/src
```
- Fork (or clone) this repo in the `~/your_workspace/src` folder by typing:
```sh 
    git clone https://github.com/badriawan/robotics_software_engineer.git
```
- Next compile and source the repository
```sh
    cd ~/your_workspace
    colcon build --packages-select module_5_assignment
    source install/setup.bash
```

- Run the demos for Assignment 2, Part 1, 2,3,4
```sh
    ros2 launch module_5_assignment task_1
    ros2 launch module_5_assignment task_2
    open task3.txt
    open task4.txt
 
```

</details>


