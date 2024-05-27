# Gathering Live Imagery, Mapping, and Pose Estimation (G.L.I.M.P.S.E.)
![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/5f221be7-6dd9-4fe7-a9ac-2a6424f06dc6)

The official repository hosting all code written for our robot, Holly, as part of the UMass Amherst Senior Design Project (SDP) 2023-24. Code is written in Python using ROS2 Humble packages for Ubuntu 22.04.3 LTS. 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/f71b7bcf-c46c-49c1-87e7-a0b400572acd)

# Workspace Navigation
Our project consists of 4 workspaces, each having their own documentation as we developed this project. Below is a brief description of each workspace we have, as well as a link to their documentations.

## Control Workspaces
These workspaces contains code that uses ROS2 Control with our robot. ROS2 Control allows us to directly configure our hardware on the robot with ROS2, so there is no need for any dedicated serial code to be written. It receives our navigations commands and converts that to individual wheel speeds, while taking odometry information from the encoders and IMU while navigating. Initial testing was done in the simulated workspace [```controlsim_ws```](https://github.com/Samalmeida1028/sdp-team-12/blob/nav_stable/controlsim_ws/README.md) and then updated with our physical robot in the [```controlphy_ws```](https://github.com/Samalmeida1028/sdp-team-12/blob/nav_stable/controlphy_ws/README.md)

## Navigation Workspaces
These workspaces contain the core code to perform navigation and tracking. Note that we decided to go without using ROS2 Control. We configure the Navigation2 stack here, along with nodes to handle searching for targets, live streaming camera feed, and performing ArUco detection. Initial testing was done in the simulated workspace [```navsim_ws```](https://github.com/Samalmeida1028/sdp-team-12/blob/nav_stable/navsim_ws/README.md) and then updated with our physical robot in the [```navphy_ws```](https://github.com/Samalmeida1028/sdp-team-12/blob/nav_stable/navphy_ws/README.md)

## PCB Documentation 
Documentation about the PCB for our project can be found at [```PCB```]() in the README.md file

Installation scripts for ROS2 Humble and Nav2 in [```rospkginstall```](https://github.com/Samalmeida1028/sdp-team-12/tree/nav_stable/navphy_ws/rospkgsinstall) directory within the [```navphy_ws```](https://github.com/Samalmeida1028/sdp-team-12/blob/nav_stable/navphy_ws/README.md) workspace. Once in that directory, run the following to install ROS2 Humble and Nav2 on your computer:
```
./ros_humble_setup.sh
./ros_humble_nav2_setup.sh
```
