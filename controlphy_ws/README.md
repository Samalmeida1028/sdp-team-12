# ROS2 Control -- Physical Robot

ROS2 Control provides a quick and easy way to control robots with different drivetrains, like differential drive, holonomic drive, etc. There are a lot of demos on the official control github repository. 

This branch aims at using the differential drive with our robot (Holly).

```controlphy_ws```: Uses the controlsim_ws but with some changes to incorporate a physical serial communication. Taken from Articulated Robotics. Work in progress...

# Execution

The demo_ws has everything necessary to visualize how ros2_control's differential drive hardware works. To run, type:

```ros2 launch diffdrive_arduino diffbot.launch.py```

This will give an error, saying the ReadByte() call has timed out. This workspace is retired in progress.
