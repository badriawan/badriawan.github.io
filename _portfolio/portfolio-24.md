---
title: "Sensor Fusion Implementation for Robot Navigation"
excerpt: Robotisim Project <br/><img src='/images/navsat.gif'>"
collection: portfolio
---


## Build a Sensor Fusion based on GPS and IMU data using EKF
<p align="center">
<img src = "/images/navsat.gif?raw=true" center=true width="55%"/>
</p>


## Brief Review

- First part of the project is to utilize the Extended Kalman Filter for Turtlebot 3 robot by fuse the IMU sensor data and GPS data only.

- Last part is just modify the value of Q and R covariance to analyze the behaviour of the mobile robot localization performance.


## Project Tree

- config
    - fusion.yaml
- include
    - ekf_lib.hpp
    - ekf_node.hpp
- launch
    - fusion.launch.py
- src
    - ekf_lib.cpp
    - ekf_node.cpp



## Solutions

- How I Built the System:

    1. Set Up the Workspace and ROS2 Packages: I started by creating the workspace and ROS2 packages, including the necessary folders such as launch, config, include, and src.

    2. Install and Add Necessary Libraries: Next, I installed and added the required libraries for the system, such as robot_localization packages, nav_msgs, tf2_msgs, rclcpp, and Eigen. I also created custom libraries as needed.

    3. Create Header Files (.hpp): I developed two header files: ekf_lib.hpp and ekf_node.hpp. The ekf_lib.hpp file contains declarations for methods related to defining the matrices H, P,Q,R,K and input. The ekf_node.hpp file includes definitions for the state, input, and class variables.

    4. Implement Source Files (.cpp): I wrote the source files ekf_lib.cpp and ekf_node.cpp. The ekf_lib.cpp file contains the EKF Matrices computation, while ekf_node.cpp serves as the ROS2 node that processes sensor input data and generates state estimation process..

    5. Create YAML Files: I created YAML files that include both the TurtleBot3 dual_ekf and the navsat configuration, this YAML file must be called as a parameter on the launch files.

    6. Turned Off the Odometry/Wheel Sensor Data: I disabled the odometry and wheel sensor data by changing the input matrix values from true to false. This step was crucial to assess the system's performance without relying on wheel encoder data, allowing the focus to be on other sensors like GPS or IMU.

    7. Generated Launch Files with Robot Localization Packages: I created launch files that included three robot_localization packages—Extended Kalman Filter (EKF) and Navsat—along with their respective YAML configuration files. These packages are responsible for fusing sensor data to improve the robot's localization accuracy.

    8. Launched All Nodes with TurtleBot3 Gazebo and RViz2: I launched all the nodes using TurtleBot3 in Gazebo for simulation and RViz2 for visualization. In RViz2, I selected the odom topics instead of base_link and odometry/global topics to observe the localization results more effectively.
   
- How I Performed the Sensor Fusion Analysis:

    1. Implementation with Higher Values of Q and R: I increased the values of the Q and R matrices compared to the default settings provided by the Navsat packages. A higher Q matrix penalizes state estimation errors more heavily, leading the controller to prioritize accuracy in the robot's position and orientation. Similarly, a higher R matrix places more penalty on the control inputs, encouraging the system to minimize the use of control efforts. By implementing higher values for both matrices, I observed how the robot became more precise in following the desired trajectory but also more conservative in its movements to save energy.

    2. Implementation with Lower Values of Q and R: I also experimented by decreasing the values of the Q and R matrices below the default settings. Lowering the Q matrix reduces the emphasis on correcting state errors, allowing the robot to tolerate deviations from the desired path. Reducing the R matrix decreases the penalty on control inputs, enabling the controller to use more aggressive control actions. This configuration helped me analyze how the robot behaves when prioritizing quicker responses over precise accuracy, potentially leading to faster but less stable movements.


## Project Summary

In the Extended Kalman Filter (EKF), the matrices Q and R are fundamental in determining the filter's performance and accuracy. The matrix Q represents the process noise covariance, capturing the uncertainties and inaccuracies in the system's dynamic model. The matrix R denotes the measurement noise covariance, reflecting the uncertainties associated with the sensor measurements. 
(Anyway , we can tuning this on fusion.yaml file)

When the system model is not perfectly known or there are external disturbances affecting the system, these uncertainties are incorporated into the filter through the matrix Q. A larger Q implies that more uncertainty is assumed in the model predictions, causing the filter to rely more heavily on the incoming measurements during the update step.

On the other hand, the matrix R models the noise present in the sensor data. If the sensors are known to be noisy or less reliable, the values in R are increased to represent this higher uncertainty. A larger R leads the filter to place less trust in the measurements, making it depend more on the model predictions.

The relationship between Q and R directly influences how the EKF balances the prediction and correction phases:

- High Q, Low R: The filter assumes that the model is highly uncertain and that the measurements are reliable. As a result, it adjusts the state estimates rapidly in response to new measurements, which may introduce more noise into the estimates if the measurements are actually noisy.

- Low Q, High R: The filter considers the model to be accurate and the measurements to be noisy. It relies more on the model predictions, leading to smoother state estimates that may lag behind rapid changes in the actual state.

Proper tuning of Q and R is crucial for optimal filter performance. If Q and R are not set appropriately, the filter may either respond too slowly to changes (if it trusts the model too much) or become too reactive to measurement noise (if it trusts the measurements too much). The values of Q and R are often determined based on knowledge of the system dynamics and the characteristics of the sensors, sometimes requiring empirical adjustment to achieve the desired filtering performance.

By carefully adjusting Q and R, the EKF can be made to provide accurate and reliable state estimates, effectively balancing the uncertainties in the model and the measurements to reflect the true state of the system as closely as possible.


## Using The Package

- Follow the next steps to replicate the outcome...

- Create the workspace:
    - `cd ~`
    - `mkdir -p your_workspace/src`
    - `cd your_workspace/src`

- Fork (or clone) this repo in the `~/your_workspace/src` folder by typing:
    - `git clone https://github.com/badriawan/robotics_software_engineer.git`

- Next compile and source the repository
    - `cd ~/your_workspace`
    - `colcon build --packages-select module_6_assignment`
    - `source install/setup.bash`


- Run the demos
    - `ros2 launch module_6_assignment fusion.launch.py`
    - `ros2 launch turtlebot3_gazebo empty_world.launch.py`
    - `rviz2`
    - `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
 

