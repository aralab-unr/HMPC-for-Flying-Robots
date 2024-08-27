# Hierarchical Model Predictive Control for online Path planning and Tracking of Flying Robots

This repository presents the ROS simulation and details of the HMPC for path planning and tracking of the flying robots: the Cube-Drone and the Quadrotor UAVs.

## Software Requirements & Setup

In order to use this package, the following components are required:

- Ubuntu 20.04
- ROS Noetic/ C++
- Gazebo 11
- Casadi ([https://web.casadi.org/](https://github.com/zehuilu/Tutorial-on-CasADi-with-CPP))

Following these instructions to install the package:

```shell
# Step 1: Create and build a catkin workspace:
mkdir -p ~/dev_ws/src
$ cd ~/dev_ws/
$ catkin_make

# Step 2: Clone this repo into your workspace
cd ~/dev_ws
git clone --recursive https://github.com/aralab-unr/Custom-controller-for-the-cube-drone.git

# Step 3: Source the environment variables
source devel/setup.bash
```
