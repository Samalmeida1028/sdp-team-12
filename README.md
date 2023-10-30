# sdp-team-12
This is the repository for Team 12's Senior Design Project (SDP)

```basic_mobile_bot_v1.launch.py``` : uses the ```basic_mobile_bot_v1.urdf``` file to put a robot in RViz to see
```basic_mobile_bot_v2.launch.py``` : uses the ```basic_mobile_bot_v1.urdf``` file to put a robot in RViz and launch a simulation in Gazebo
```basic_mobile_bot_v3.launch.py``` : performs sensor fusion for odometry and IMU and publishes to a new topic in a smalltown.world simulation
```basic_mobile_bot_v4.launch.py``` : shows a smalltown.world simulation for the robot with LiDAR on it using ```basic_mobile_bot_v2.urdf```

```basic_mobile_bot_v5.launch.py``` : uses Nav2 to localize the robot using LiDAR with and without SLAM in a smalltown world using the Costmap API

Use ```rqt_robot_steering``` and change the topic to ```cmd_vel``` to use a GUI for moving the robot in Gazebo. 

Things I did so far: 
1. Set up a URDF and view it in RViz
2. Set up a SDF for Gazebo to simulate the URDF with rqt_robot_steering
3. Set up a smalltown.world to simulate the URDF in a world instead of bare Gazebo
4. Set up a LiDAR on the URDF and simulated that in the smalltown.world
5. Set it up with Nav2 to do navigation from point A to B with a map