# Physical Navigation Workspace

Installation scripts for ROS2 Humble and Nav2 in ```rospkginstall``` directory. Once in that directory, run the following to install ROS2 Humble and Nav2 on your computer:
```
./ros_humble_setup.sh
./ros_humble_nav2_setup.sh
```

## Physical Navigation using ROS2 Navigation Stack
This branch contains a number of packages:
- ```basic_mobile_robot``` : all the navigation necessities. Contains URDF file for the robot, SLAM and EKF configuration files, and navigation launch script
- ```navigator```: navigation nodes to work with target tracking, search behaviors, and publishing odometry
- ```py_img_stream```: performs our core computer vision to recognize targets and perform pose estimation on them
- ```py_serial```: handles all serial communication between ROS2 and our hardware
- ```gui```: an interactive Graphical User Interface (GUI) for the user
- ```obs_nodes```: Open Broadcast Service (OBS) websocket node that launches with our navigation to live stream camera feed
- ```rpi_pico```: backup code for our navigation and target tracking Picos
- ```scripts```: contains Bash scripts to start our robot processes. Used by our GUI

## Starting the Robot
After flipping the switch on and turning on the computer on Holly, you are able to Secure Shell (SSH) in using the command below:
```
ssh -Y sdpteam12@IPADDR
```
The ```IPADDR``` field is what shows up when the computer is connected to your private network (mobile hotspot or other private network). Note that this will not work on eduroam. 

Then, type the following into the terminal:
```
./run.sh
```
This script is located in the home directory of the on-board computer of the robot. This will launch the GUI. From here, clicking on ```Launch Navigation``` will launch all required nodes for navigation and tracking. You will be able to enter targets into the GUI as follows:

```[[ID, side length, time in seconds], [ID, side length, time in seconds]]```

This allows the user to specify a target ID of variable size for variable time of tracking. Larger targets can be seen from further away. 

## Behind the scenes: Navigation
The navigation stack is the most complicated part of our design project. The list of things involved, in order, goes as follows:
- Robot Localisation
- Simultaneous Localization And Mapping (SLAM)
- Coordinate Transforms
- Nav2 Servers
- Serial Communication

## Robot Localisation
The robot is localized using an Extended Kalman Filter (EKF). This node will take in all sources of odometry we provide it, and fuse them together into an odometry topic. This topic's information is read in by the navigation modules to accurately place the robot in the environment. Sources we use are:
- LiDAR ranges
- Wheel encoders
- Inertial Management Unit (IMU)

Suppose you know the starting point of an object. You wish to know where the object will be in the future. To do this, you will make an estimate. But how do you make that estimate? You will need to know something about the system, like the dynamics of it. If you know how the object is moving, you will be able to mathematically obtain its location in the future, given you know how long into the future you are extrapolating. But in the real world, this measure of the object's movement is noisy. As the time into the future increases, so does the uncertainty associated with the location of the object. 

But, it is still possible to make an estimate of the object's location using probability and odometry measurements. This is what the Kalman Filter does. It uses the covariances associated with each source of odometry, and models the object's future locations using a probabilistic approach. This process does involve some level of noise, but will be accurate to a margin. 

An extended Kalman filter is used when the measurements are noisy and there is uncertainty associated with the accuracy of each source of odometry. Since our wheel encoders do not account for wheel slippage, and our IMU can be noisy, and our LiDAR is not the best quality, it is possible that one or more of these readings are wrong. Our EKF node accounts for this error using covariances of each source and fuses all the odometry information together. 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/ba99deaf-5d88-48d2-91d7-bec8f73e0685)

Shown below is a connected tree showing the flow of information for our localization using EKF. 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/33ba2bae-2a5e-466b-9848-0de6fb2b136f)

## SLAM
SLAM uses LiDAR ranges to localize the robot and navigate while mapping. It is a particle filter that probabilistically approximates the robot location using the measurements it receives. Our robot performs SLAM to generate the map and uses the joint odometry information from EKF to localize itself. Shown below is a SLAM generated map of the UMass Amherst M5! 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/688b8ccc-6d9e-4b77-b894-d1b22d476c36)

## Coordinate Transforms
Coordinate Transforms are how ROS understands movement. When the robot moves in the real world, ROS needs to know that is happening too. It does so by mapping the coordinates of each component of the robot and the environment relative to each other. In our robot description file, we have joints and links. Joints are pieces of the robot that can move and rotate. They connect links together (eg: Left wheel joint, Right wheel joint). Links are pieces of the robot that cannot move (eg: Base, LiDAR, Wheel). Each link and joint is mapped together using the ```robot_state_publisher``` and ```joint_state_publisher``` nodes provided by ROS. 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/32681a65-1e49-4259-8362-fd9fdf01dd2e)

But we also need a coordinate transform from the odometry of the robot to the base of the robot. This tells us where the robot is. So a transform is published by EKF. Same way, we need to know where the robot is in the map generated, so another transform is published from SLAM. Shown below is the complete coordinate transform tree. 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/f7c0b655-bd9a-48ee-a253-f3ebd0e7bf9d)

So this now connects each part of the environment with everything else in ROS. 

## Nav2 Servers
So the robot is initialized, the transforms are set, and SLAM + EKF are working. Now how does the navigation work? First, Nav2 sets up a Behavior Tree. This contains all the behaviors the robot will perform. First, it tries to compute a path to the goal and smooth it. This happens at a rate of 1 Hz. As soon as the path is computed, the follow instructions are given. Now, let's say the navigation is unsuccessful, due to some obstacle or noise. There are recovery behaviors that will be executed in a Round Robin fashion. First, the global and local costmaps are set. Then, ROS tries to navigate again. If that fails, then the robot is spun 90 degrees, and ROS tries navigating again. If that fails, it waits 5 seconds and tries again. If that too fails, it backs up about 30 cm and tries again. If everything fails, it reruns the whole sequence of navigation and recovery up to 6 times before aborting navigation. 

With this tree, Nav2 launches the ```behavior_server``` and ```smoother_server```. The behavior server handles all the behaviors specified in the tree defined above, while the smoother server intelligently smooths the paths it receives to avoid any obstacles to some radius. This is done using the ```ConstrainedSmoother``` from the Nav2 API. With all this, the simplified Nav2 block diagram looks like below:

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/1ee7b45f-0c73-4795-837a-6b310e492032)

Next, the ```planner_server``` and ```controller_server``` are launched. The planner uses the ```SmacHybridPlanner``` from Nav2 API and runs A* search to find the fastest path to goal. The controller computes the ```cmd_vel``` Twist messages and publishes to the ```/cmd_vel_nav``` topic. This is used by the serial communication node. So the block diagram updates like so:

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/b6bfc5bb-e685-439f-bf84-2fd3a542b197)

Next, the certerpiece of the navigation system comes online. This is the ```bt_navigator``` node, which takes in all the outputs and goal pose information and keeps track of the navigation. It uses the ```bond``` timer to sync all the servers together and run navigation smoothly. The final block diagram is as follows: 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/4a87bee6-890b-4f12-bda9-f1737b721793)

## Serial Communication
On the serial side, the ```/cmd_vel_nav``` topic is subscribed to and the Twist messages are converted into a linear and linear-rotational vector. This vector is then sent to the Nav Pico, which will then communicate with the motors and set their velocities accordingly. It will then read the wheel and IMU odometry data, and send it back to ROS. And thus, we have navigation with the physical robot. 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/f5d28e58-e7bc-4cf5-89b7-cca8e2596e42)

For target tracking, we take the marker location, published in ```/marker_location``` topic, as an offset from the center, and send that to our target tracking Pico. This will then command the servos to center the camera at the center of the marker. We predict the motion of the marker using the offset's velocity and position to allow smooth camera movement. This allows us to perform target tracking when CPU resources are constrained, and we cannot calculate the pose of the marker fast (under 20 Hz). With our computer, however, we are able to calculate the pose at 30 Hz and above easily. 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/ab3f6ce5-edd7-491d-afe9-5e546c6bcd06)

