---
title: "Deep-Q-Network for Mobile Robotics Navigation"
excerpt: Research Project <br/><img src='/images/drl.gif'>"
collection: portfolio
---

## Reward Shaping for Deep Reinforcement Learning in Mobile Robot Navigation
<p align="center">
<img src = "/images/drl.gif?raw=true" center=true width="55%"/>
</p>


## Abstract

In recent years, various robot navigation methods have been proposed, including the use of Simultaneous Localization and Mapping (SLAM), path planning algorithms such as A* or Dijkstra, and obstacle avoidance methods like Artificial Potential Fields. Navigation methods are crucial for mobile robots to navigate autonomously. However, challenges remain in incorporating various constraints and diverse sensor data into the navigation process, especially when maps of the environment are unavailable. The uncertainty of such environments has led to the development of navigation methods using Deep Reinforcement Learning (DRL). However, to solve the navigation problem effectively, DRL algorithms require reward shaping to generate suitable reward functions with limited observation data.

In this research, we propose reward shaping techniques for Deep Q-Network (DQN) and Quantile Regression DQN (QR-DQN), which are DRL algorithms used to perform navigation for a mobile robot. The goal of the algorithm is to achieve a specific position with optimum accuracy in a 2-dimensional space, using only positioning and distance data.

We compared positive and negative reward functions to train the system and evaluated two types of robot environments. We successfully achieved the best positioning performance with 87.3% accuracy in environments without obstacles and 74.5% accuracy in environments with obstacles.

**Keywords**: Navigation, Deep Reinforcement Learning, Reward Shaping


## Introduction

Autonomous navigation is a fundamental capability for mobile robots operating in dynamic and uncertain environments. Traditional navigation methods have made significant progress, but they often rely on pre-existing maps or extensive sensor arrays to function effectively. The limitations of these methods in unknown or changing environments necessitate alternative approaches.

Deep Reinforcement Learning (DRL) offers a promising solution by enabling robots to learn optimal navigation strategies through interactions with the environment. However, designing an effective reward function—known as reward shaping—is critical to the success of DRL algorithms, especially when limited observation data is available.

---

## Background

### Traditional Navigation Methods

- **Simultaneous Localization and Mapping (SLAM)**: Builds a map of an unknown environment while simultaneously tracking the robot's location.
- **Path Planning Algorithms**: Algorithms like A* and Dijkstra find the shortest path between two points but require a known map.
- **Obstacle Avoidance Techniques**: Methods like Artificial Potential Fields use virtual forces to repel the robot from obstacles.

### Challenges in Uncertain Environments

- **Lack of Pre-existing Maps**: Robots cannot rely on prior knowledge and must navigate using real-time data.
- **Sensor Limitations**: Diverse and high-quality sensor data may not be available, restricting observation capabilities.
- **Dynamic Obstacles**: Environments with moving obstacles add complexity to the navigation task.

---

## Deep Reinforcement Learning

### Deep Q-Network (DQN)

DQN combines Q-learning with deep neural networks to handle high-dimensional state spaces. It estimates the optimal action-value function, enabling the agent to select actions that maximize cumulative rewards.

### Quantile Regression DQN (QR-DQN)

QR-DQN extends DQN by modeling the distribution of possible future rewards instead of just the mean. This approach provides a more robust learning process by considering the uncertainty in value estimates.

---

## Reward Shaping

Designing an effective reward function is crucial for guiding the learning process in DRL.

### Positive Reward Function

- **Description**: Provides positive feedback when the robot moves closer to the target.
- **Purpose**: Encourages behaviors that reduce the distance to the goal.

### Negative Reward Function

- **Description**: Applies penalties when the robot moves away from the target or encounters obstacles.
- **Purpose**: Discourages undesirable actions that hinder navigation success.

---

## Methodology

### Environment Setup

- **Simulation Space**: A 2D environment where the robot navigates to a target position.
- **Robot Capabilities**: Limited to positioning and distance sensing to simulate minimal observation conditions.
- **Two Scenarios**:
  - **Without Obstacles**: An open environment to evaluate basic navigation performance.
  - **With Obstacles**: Includes static obstacles to test obstacle avoidance capabilities.

### Training Process

1. **Initialization**: Set up the environment and initialize the DRL agent.
2. **Reward Function Assignment**: Implement either the positive or negative reward function.
3. **Learning Phase**: Allow the agent to interact with the environment and learn from feedback.
4. **Evaluation**: Test the trained agent to assess navigation accuracy.

---

## Experiments and Results

### Performance Without Obstacles

- **Accuracy Achieved**: 87.3%
- **Observations**:
  - The agent effectively learned to navigate directly to the target.
  - The negative reward function led to faster convergence during training.

### Performance With Obstacles

- **Accuracy Achieved**: 74.5%
- **Observations**:
  - The presence of obstacles introduced additional complexity.
  - The agent learned to avoid obstacles while approaching the target.
  - There was a noticeable decrease in accuracy compared to the obstacle-free environment, highlighting the challenge of obstacle avoidance.

---

## Conclusion

The research demonstrates that reward shaping is vital for training DRL agents in mobile robot navigation tasks, especially with limited sensory data. The negative reward function proved more effective in guiding the agent towards the target while avoiding undesirable behaviors.

By comparing DQN and QR-DQN algorithms using different reward functions, we achieved high positioning accuracy in both simple and complex environments. This work contributes to the development of autonomous navigation systems that can operate in uncertain and uncharted environments.

---

## Future Work

- **Extended Sensor Inputs**: Incorporate additional sensors like LiDAR or cameras to enhance perception.
- **Dynamic Obstacles**: Test the agent's performance in environments with moving obstacles.
- **Real-world Implementation**: Transfer the trained models to physical robots to evaluate real-world applicability.
- **Algorithm Optimization**: Explore other DRL algorithms or hybrid approaches to improve learning efficiency.

---


# How I Created System for TurtleBot Gym Environment

This documentation provides an in-depth explanation of the `TurtleBot` class, a custom OpenAI Gym environment for simulating a TurtleBot robot using the PyBullet physics engine.

## Overview

The `TurtleBot` class is a custom environment that extends `gym.Env` to simulate a TurtleBot robot for reinforcement learning tasks. It uses PyBullet for physics simulation and is compatible with RL libraries like `stable_baselines3`.

## Prerequisites

- **Python 3.x**
- **Libraries**:
  - `gym`
  - `numpy`
  - `pybullet`
  - `pybullet_data`
  - `math`
  - `random`
  - `time`
  - `os`

Install the required libraries using:

```bash
pip install gym numpy pybullet pybullet_data
```

## Class Structure

### Imports

```python
import gym
from gym import spaces
import math
import numpy as np
import random
import pybullet as p
import time
import pybullet_data
import os
```

- **gym**: Base library for OpenAI Gym environments.
- **spaces**: Used to define action and observation spaces.
- **math**, **numpy**, **random**: For mathematical operations.
- **pybullet**: Physics simulation engine.
- **time**: For controlling simulation timing.
- **pybullet_data**: Provides access to data paths for PyBullet.
- **os**: Interacting with the operating system.

### Class Definition

```python
class TurtleBot(gym.Env):
    # Class methods go here
```

The `TurtleBot` class inherits from `gym.Env` to create a custom environment.

#### Initialization (`__init__`)

```python
def __init__(self, sim_active):
    super(TurtleBot, self).__init__()
    self.sim_status = sim_active
    if self.sim_status == 1:
        physicsClient = p.connect(p.GUI)
    else:
        physicsClient = p.connect(p.DIRECT)
    
    self.MAX_EPISODE = 10000000
    self.x_threshold = 10

    high = np.array([self.x_threshold, self.x_threshold], dtype=np.float32)
    self.action_space = spaces.MultiDiscrete([3, 3])
    self.observation_space = spaces.Box(-high, high, dtype=np.float32)

    self.steps_left = self.MAX_EPISODE
    self.state = [0, 0]
    self.orientation = [0, 0, 0, 0]
    self.x_target = [5.5, 5.5]
    self.start_simulation()
```

- **Simulation Status**: Determines if the simulation runs in GUI mode (`p.GUI`) or headless mode (`p.DIRECT`).
- **Environment Parameters**:
  - `MAX_EPISODE`: Maximum number of steps per episode.
  - `x_threshold`: Threshold for the state space.
- **Spaces**:
  - `action_space`: Defined as `MultiDiscrete([3, 3])`, allowing for combinations of discrete actions.
  - `observation_space`: A continuous space defined by `Box`.
- **Initial State**:
  - `steps_left`: Countdown of steps remaining in the episode.
  - `state`: Initial position `[0, 0]`.
  - `orientation`: Initial orientation in quaternion.
  - `x_target`: Goal position `[5.5, 5.5]`.
- **Simulation Start**: Calls `start_simulation()` to initialize the simulation environment.

#### Step Function (`step`)

```python
def step(self, action):
    # Action interpretation
    if action[0] == 0:
        speed_left = 1
    elif action[0] == 1:
        speed_left = 0
    else:
        speed_left = -1

    if action[1] == 0:
        speed_right = 1
    elif action[1] == 1:
        speed_right = 0
    else:
        speed_right = -1

    # Apply action to motors
    p.setJointMotorControl2(self.boxId, 0, p.VELOCITY_CONTROL,
                            targetVelocity=speed_left * 20, force=1000)
    p.setJointMotorControl2(self.boxId, 1, p.VELOCITY_CONTROL,
                            targetVelocity=speed_right * 20, force=1000)

    # Update simulation
    p.stepSimulation()
    time.sleep(1./240.)

    # Read sensor data
    linkState = p.getLinkState(self.boxId, 4, computeLinkVelocity=1,
                               computeForwardKinematics=1)
    linkWorldPosition = linkState[0]
    linkWorldOrientation = linkState[1]

    # Update state and orientation
    self.orientation = list(linkWorldOrientation)
    self.state = [linkWorldPosition[0], linkWorldPosition[1]]

    # Check if the episode is done
    done = bool(self.steps_left < 0)

    # Calculate reward
    if not done:
        error = np.array(self.state) - np.array(self.x_target)
        reward = -np.linalg.norm(error) ** 2
    else:
        reward = -100

    # Update steps left
    if not done:
        self.steps_left -= 1
        self.act = action
        self.cur_done = done

    # Return observation, reward, done, and info
    return np.array([self.state]), reward, done, {}
```

- **Action Interpretation**: Maps discrete actions to motor speeds.
- **Motor Control**: Uses `setJointMotorControl2` to apply velocities to the left and right wheels.
- **Simulation Update**: Advances the simulation by one timestep.
- **State Update**: Reads the robot's position and orientation.
- **Termination Condition**: Episode ends when `steps_left` reaches zero.
- **Reward Calculation**:
  - Negative squared Euclidean distance to the target position.
  - Penalty of `-100` if the episode is done.
- **Return Values**:
  - `observation`: Current state as a NumPy array.
  - `reward`: Calculated reward.
  - `done`: Boolean indicating if the episode is finished.
  - `info`: An empty dictionary (can be used for additional data).

#### Simulation Start (`start_simulation`)

```python
def start_simulation(self):
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # Create collision object
    self.cuid = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3])
    self.mass = 0  # Static box
    self.collision = p.createMultiBody(self.mass, self.cuid, -1, [-1, 0.2, 0])

    # Load plane
    planeId = p.loadURDF("plane.urdf")

    # Load TurtleBot
    startPos = [self.state[0], self.state[1], 0.0]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    self.boxId = p.loadURDF("turtlebot.urdf", startPos, startOrientation)
```

- **Physics Setup**:
  - Sets the search path for URDF files.
  - Sets gravity in the simulation.
- **Collision Object**:
  - Creates a static collision box (could represent an obstacle).
- **Environment Setup**:
  - Loads a plane to serve as the ground.
- **Robot Initialization**:
  - Loads the TurtleBot URDF at the starting position and orientation.

#### Reset Function (`reset`)

```python
def reset(self):
    p.resetSimulation()
    self.start_simulation()
    self.state = [0, 0]
    self.steps_left = self.MAX_EPISODE
    return np.array([self.state])
```

- **Simulation Reset**:
  - Resets the entire simulation environment.
  - Re-initializes the simulation.
- **State Reset**:
  - Resets the robot's state and steps left.
- **Return**:
  - Returns the initial observation.

#### Render Function (`render`)

```python
def render(self, mode='human'):
    print(f'State: {self.state}, Action: {self.act}, Done: {self.cur_done}')
```

- **Rendering**:
  - For the 'human' mode, it prints the current state, action taken, and whether the episode is done.
  - Can be expanded for graphical rendering if needed.

## Usage

To use the `TurtleBot` environment:

1. **Import the Environment**:

   ```python
   from your_module import TurtleBot
   ```

2. **Initialize the Environment**:

   ```python
   env = TurtleBot(sim_active=1)  # Use sim_active=0 for headless mode
   ```

3. **Interaction Loop**:

   ```python
   obs = env.reset()
   done = False
   while not done:
       action = env.action_space.sample()  # Replace with your agent's action
       obs, reward, done, info = env.step(action)
       env.render()
   ```

4. **Close the Environment** (if necessary):

   ```python
   env.close()
   ```

## Notes

- **Action Space**: The action space is a `MultiDiscrete([3, 3])`, meaning each action is a tuple where each element can be `0`, `1`, or `2`. These correspond to moving the left and right wheels forward, stopping, or backward.
- **Observation Space**: The observation is a 2D position of the robot within the defined thresholds.
- **Reward Function**: Encourages the robot to minimize the distance to the target position.

## Troubleshooting

- **PyBullet Connection Error**: Ensure that you are not trying to establish multiple GUI connections. Use `p.disconnect()` before reconnecting if needed.
- **URDF Files Not Found**: Make sure that `turtlebot.urdf` and other URDF files are located in the search path or provide the full path.
- **Incorrect Action Dimensions**: Ensure that the actions provided to `env.step()` match the defined action space.


# How I Trained the Agent Script

This documentation provides an overview and explanation of the Python script designed to train and run a Deep Q-Network (DQN) agent using the `stable_baselines3` library in a custom `TurtleBot` environment.


## Overview

The script:

- **Loads or trains** a DQN agent to interact with the `TurtleBot` environment.
- **Performs inference** using the trained model to make the TurtleBot navigate.
- **Optionally collects data** such as the TurtleBot's position during navigation.

## Prerequisites

- **Python 3.x**
- Required Python libraries:
  - `gym`
  - `collections`
  - `math`
  - `numpy`
  - `pandas`
  - `pathlib`
  - `stable_baselines3`
- Custom modules:
  - `Thesisgym1` containing the `TurtleBot` environment.

Ensure all libraries are installed:

```bash
pip install gym collections math numpy pandas pathlib stable_baselines3
```

## Script Structure

### Imports

```python
import gym
import collections
import math
import numpy as np
import pandas as pd
from pathlib import Path

from Thesisgym1 import TurtleBot
from stable_baselines3 import DQN
from stable_baselines3.common.env_util import make_vec_env
```

- **Standard Libraries**: Import essential Python libraries for mathematical operations and data handling.
- **Custom Environment**: Import `TurtleBot` from the `Thesisgym1` module.
- **Reinforcement Learning Library**: Import DQN algorithm and utilities from `stable_baselines3`.

### Simulation Flag

```python
SIM_ON = 1
```

- **SIM_ON**: A flag to indicate whether the simulation is active (`1`) or not.

### Main Execution

```python
if __name__ == "__main__":
    # Code goes here
```

- Ensures that the script runs only when executed directly, not when imported as a module.

#### Environment Initialization

```python
env = TurtleBot(SIM_ON)
# print("rad_theta =", env.rad_theta)
```

- **Environment Setup**: Initializes the `TurtleBot` environment with the simulation flag.
- **Optional Print Statement**: Can display the `rad_theta` attribute for debugging.

#### Model Loading and Training

```python
# Load an existing model
model = DQN.load("thesis_5M_negreward")

Alternatively, train a new model (commented out)
model = DQN("MlpPolicy", env, verbose=1, tensorboard_log="./thesis_data_negreward_5M/")
model.learn(total_timesteps=5000000)
model.save("thesis_5M_negreward")
print("I'm done baby!")
```

- **Model Loading**: Loads a pre-trained model named `thesis_5M_negreward`.
- **Model Training**: (Commented out) Shows how to create, train, and save a new DQN model.
  - **Policy**: Uses Multi-Layer Perceptron policy (`"MlpPolicy"`).
  - **Logging**: TensorBoard logging is set up for monitoring training progress.

#### Agent Interaction Loop

```python
obs = env.reset()
while True:
    action, _states = model.predict(obs, deterministic=True)
    print(action)
    obs, reward, done, info = env.step(action)
    env.render()
    if done:
        obs = env.reset()
```

- **Reset Environment**: Begins with resetting the environment to obtain the initial observation.
- **Infinite Loop**: The agent continuously interacts with the environment:
  - **Action Prediction**: The model predicts the next action based on the current observation.
  - **Environment Step**: Executes the action in the environment.
  - **Rendering**: Updates the visual display of the environment.
  - **Episode Completion**: Checks if the episode is finished and resets if necessary.

#### Data Collection (Optional)

```python
position = pd.DataFrame(columns=["x"])
for x in range(5000):  # Approx. 35 seconds
    action, _states = model.predict(obs, deterministic=True)
    print(action)
    obs, reward, done, info = env.step(action)
    env.render()
    position.loc[len(position.index)] = [obs[0]]
    if done:
        obs = env.reset()
print(position)
filepath = Path("position_neg.csv")
filepath.parent.mkdir(parents=True, exist_ok=True)
position.to_csv(sep=';', path_or_buf=filepath)
```

- **Position DataFrame**: Creates a DataFrame to store position data.
- **Data Collection Loop**: Runs for 5000 iterations to collect data.
- **Data Saving**: Saves the collected data to a CSV file named `position_neg.csv`.

## Usage

### Running the Script

1. **Ensure Dependencies**: All required libraries and modules are installed.
2. **Set Up Environment**: The `Thesisgym1` module and `TurtleBot` environment must be properly configured.
3. **Model Availability**: Have the `thesis_5M_negreward` model file in the script directory or specify the correct path.
4. **Execute Script**:

   ```bash
   python3 your_script_name.py
   ```

### Training a New Model

- Uncomment the model training section:

  ```python
  model = DQN("MlpPolicy", env, verbose=1, tensorboard_log="./thesis_data_negreward_5M/")
  model.learn(total_timesteps=5000000)
  model.save("thesis_5M_negreward")
  print("I'm done baby!")
  ```

- Comment out the model loading line:

  ```python
   model = DQN.load("thesis_5M_negreward")
  ```

### Collecting Position Data

- Ensure that you have sufficient storage space for the output CSV file.

## Notes

- **Simulation Toggle**: Modify the `SIM_ON` variable to switch between simulation modes if applicable.
- **TensorBoard Logging**: Use TensorBoard to visualize training progress by running:

  ```bash
  tensorboard --logdir=./thesis_data_negreward_5M/
  ```

- **Custom Environment**: The `TurtleBot` environment should comply with OpenAI Gym interface standards.

## Troubleshooting

- **Module Not Found Errors**: Ensure all custom modules and dependencies are correctly installed and accessible.
- **Environment Errors**: Verify that the `TurtleBot` environment is correctly set up and compatible with the script.
- **File Paths**: Check that all file paths (for models and data files) are correct and accessible.


