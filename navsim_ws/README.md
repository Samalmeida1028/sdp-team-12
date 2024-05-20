# Simualated Navigation using the ROS2 Navigation Stack

In this branch:
- sim_bot

Before running, make sure to have ROS2 Humble and Nav2 installed on your computer. 

## sim_bot Package

There are a bunch of launch files that all do different things. Below is an overview. 

```urdf_visualizer.launch.py``` : uses the ```basic_mobile_bot_v1.urdf``` file to put a robot in RViz to see

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/f2655747-9df3-4d7a-8c01-b5f924094494)

```gazebo_simulator.launch.py``` : uses the ```basic_mobile_bot_v1.urdf``` file to put a robot in RViz and launch a simulation in Gazebo

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/2f461890-2130-421e-a78f-1760064bc30a)

```sensor_fusion_odom.launch.py``` : performs sensor fusion for odometry and IMU and publishes to a new topic in a smalltown.world simulation

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/b1a9518e-708f-4a0c-a4dc-ad69207ac9ec)

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/109f8b09-79f5-4bcc-ae27-fe373f6088df)

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/53b5e9a0-9ef9-4047-8077-09cd27f624f5)

```lidar_config.launch.py``` : shows a smalltown.world simulation for the robot with LiDAR on it using ```basic_mobile_bot_v2.urdf```

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/85201a7e-6a0a-459f-9ac2-8188bb63fb1b)

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/fc216024-2f4b-4d31-904d-3e02c310ab27)

```nav_env.launch.py``` : uses Nav2 to localize the robot using LiDAR with and without SLAM in a smalltown world using the Costmap API

Without SLAM: 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/ada860b0-6ff9-48e8-96d2-718f4ef2837d)

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/7450e38e-e833-4e77-a911-9ae1eae72a2d)

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/953ec8ca-31a0-4ec7-9469-5f42a1d89273)

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/7b42a7b6-87f5-4556-b241-89a4517796e8)

With SLAM: 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/0b0820ba-6ebf-4e3d-bb77-9e716cc6b467)

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/640f0523-9a59-4c94-a101-6ae795f46254)

Use ```rqt_robot_steering``` and change the topic to ```cmd_vel``` to use a GUI for moving the robot in Gazebo. 

