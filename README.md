# sdp-team-12
This is the repository for Team 12's Senior Design Project (SDP)

## Navigation

In this branch, there are 2 main packages:
- basic_mobile_robot
- navigator

Before running, make sure to run both ```ros_humble_setup.sh``` and ```ros_humble_nav2_setup.sh``` scripts in ```basic_mobile_robot/rospkginstall``` directory, if you do not have ROS2 Humble and/or Nav2 installed

## basic_mobile_robot Package

There are a bunch of launch files that all do different things. Below is an overview. 

```urdf_visualizer.launch.py``` : uses the ```basic_mobile_bot_v1.urdf``` file to put a robot in RViz to see \

```gazebo_simulator.launch.py``` : uses the ```basic_mobile_bot_v1.urdf``` file to put a robot in RViz and launch a simulation in Gazebo \

```sensor_fusion_odom.launch.py``` : performs sensor fusion for odometry and IMU and publishes to a new topic in a smalltown.world simulation \

```lidar_config.launch.py``` : shows a smalltown.world simulation for the robot with LiDAR on it using ```basic_mobile_bot_v2.urdf``` \

```nav_env.launch.py``` : uses Nav2 to localize the robot using LiDAR with and without SLAM in a smalltown world using the Costmap API \

Use ```rqt_robot_steering``` and change the topic to ```cmd_vel``` to use a GUI for moving the robot in Gazebo. 

## navigator Package

This package has code to publish to ```/cmd_vel``` topic through keyboard input as well as publish a random pose to check if navigation works

Points to navigate to are in the the form of ```PoseStamped``` messages. They are published to the ```/goal_pose``` topic. Once this is done, ```/cmd_vel_nav``` will output many ```Twist``` messages for the robot to get to the goal state. 

