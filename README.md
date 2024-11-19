# ROS2 Voronoi Multi-Robot Exploration

> **⚠️ Work in Progress**  
> This setup is currently non-functional and under active development. 

---

## Overview

This project aims to use a Voronoi-partition-based exploration algorithm to enable multiple robots to autonomously navigate and explore an environment, generating a map through TurtleBot simulations.

---

## Requirements

- **ROS2 Humble**
- **TurtleBot Packages**
  - `turtlebot4` (for TurtleBot 4)
  - `turtlebot3` (for TurtleBot 3)

---

## Installation

Clone and build the workspace:

```bash
git clone https://github.com/akifbayram/ros2_voronoi.git
cd ~/ros2_voronoi
colcon build
```

---

## TurtleBot4 Simulation

The launch file `tb4.launch.py` may be modified for one or multiple robots.

1. **Launch Simulation**:
   ```bash
   cd ~/ros2_voronoi
   . install/setup.bash
   ros2 launch voronoi tb4.launch.py
   ```

2. **Start Exploration**:
   ```bash
   source /etc/turtlebot4/setup.bash
   cd ~/ros2_voronoi
   . install/setup.bash
   ros2 run voronoi voronoi_tb4
   ```

---

## TurtleBot3 Simulation

1. **Launch Simulation**:
   ```bash
   cd ~/ros2_voronoi
   . install/setup.bash
   ros2 launch voronoi tb3.launch.py
   ```

2. **Start Exploration**:
   ```bash
   source /etc/turtlebot4/setup.bash
   cd ~/ros2_voronoi
   . install/setup.bash
   ros2 run voronoi voronoi_tb3
   ```

---

### Known Issues/Challenges

- **Multi-Robot Communication**: Users may experience intermittent issues with launching SLAM, Nav2, or RViz for additional TurtleBot4 units.

    - [Nav2 Stalling on Multiple Robots](https://github.com/ros-navigation/navigation2/issues/4350)
    - [SLAM with namespaced Robots](https://github.com/turtlebot/turtlebot4/issues/159)
    - [gazebo ignition simulation crashes when trying to simulate multiple turtlebot4 ](https://github.com/turtlebot/turtlebot4_simulator/issues/60)


## **Acknowledgments**

This project draws inspiration and code from the following repositories:

1. [**abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot**](https://github.com/abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot):  
   For map merging, path planning, following, and obstacle avoidance.

2. [**Connected-and-Autonomous-Systems-Lab/Voronoi**](https://github.com/Connected-and-Autonomous-Systems-Lab/voronoi):  
   Provides the Voronoi-based exploration algorithm used in this project.  
   - This repository is itself adapted from [**Peace1997/Voronoi_Based_Multi_Robot_Collaborate_Exploration_Unknow_Enviroment**](https://github.com/Peace1997/Voronoi_Based_Multi_Robot_Collaborate_Exploration_Unknow_Enviroment/tree/master)