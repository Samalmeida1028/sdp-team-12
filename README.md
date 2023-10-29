# sdp-team-12
This is the repository for Team 12's Senior Design Project (SDP)

```basic_mobile_bot_v1.launch.py``` : uses the ```basic_mobile_bot_v1.urdf``` file to put a robot in RViz to see
```basic_mobile_bot_v2.launch.py``` : uses the ```basic_mobile_bot_v2.urdf``` file to put a robot in RViz and launch a simulation in Gazebo
```basic_mobile_bot_v3.launch.py``` : performs sensor fusion for odometry and IMU and publishes to a new topic in a smalltown.world simulation
```basic_mobile_bot_v4.launch.py``` : shows a smalltown.world simulation for the robot 

Use ```rqt_robot_steering``` and change the topic to ```cmd_vel``` to use a GUI for moving the robot in Gazebo. 