# ROS2 Control -- Simulated

ROS2 Control provides a quick and easy way to control robots with different drivetrains, like differential drive, holonomic drive, etc. There are a lot of demos on the official control github repository. 

This branch aims at using the differential drive with our robot (Holly). There are 2 workspaces. 

```controlsim_ws```: Uses differential drive hardware defined to print out individual wheel speeds from Twist messages received. Uses example 2 demo from ros2_control_demos github repository with our SDP \

# Execution

The workspace has everything necessary to visualize how ros2_control's differential drive hardware works. To run, type:

```ros2 launch ros2_control_demo_example_2 diffbot.launch.py```

This will fire up RViz with Holly and also print out wheel speeds it receives on the terminal. 

Then, we want to send it Twist messages to see ros2_control convert them to wheel speeds. But the hardware interface used has its own topic to receive Twist messages, so we need to remap ```/cmd_vel``` to that topic. In this case, we are remapping to ```/diffbot_base_controller/cmd_vel_unstamped```. Type:

```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped```

Below is an image of what you will see once everything fires up correctly.

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/150167579/4877e804-8d8a-4dab-9530-07503edd4960)

When you drive the robot around, you will see the print statements update with speed commands. 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/150167579/298a4192-a207-4f46-b02f-1c84ecb12a70)

These wheel speeds are what we will use on the physical robot to drive it with Nav2!
