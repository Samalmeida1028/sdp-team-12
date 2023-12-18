# Senior Design Project 2023
This is the repository for Team 12's Senior Design Project (SDP)

## Physical Navigation using ROS2 Navigation Stack
This branch contains a number of packages:
- ```basic_mobile_robot``` : all the navigation necessities. Contains installation scripts for ROS2 Humble and Nav2 in ```rospkginstall``` directory. Contains URDF file for the robot, SLAM and EKF configuration files, and navigation launch script
- ```navigator```: to be finished. Will take goal pose and relay wheel velocity commands to motors
- ```py_img_pub```: contains the target tracking code
- ```serial_handler```: contains a node that handles all serial communication between ROS and our Pico

## Ports:
RPLiDAR: ```/dev/ttyUSB0``` \
Nav Pico: ```/dev/ACM1``` \
Nav Pico REPL: ```/dev/ACM0``` \
Tracking Pico: ```/dev/ACM3``` \
Tracking Pico REPL: ```/dev/ACM2``` 

## Running Navigation
Once all the required ROS2 packages are installed, make sure the Nav Pico is connected to the computer and the correct port is verified. Then, open a REPL in VS Code to see any input coming from ROS. Then, run the ```nav_physical.launch.py``` script. This will fire up the Nav stack and open RViz showing the robot and SLAM output.

When the launch file is executed, you will see that the REPL initially has a bunch of empty lists. This is because empty commands are coming through at the moment. Once you set the initial and goal poses in RViz, you will see a red path calculated and the REPL output will be filled with values for wheel velocities. The robot will also move. 

Below is what your screen should look like when you have CircuitPython, the terminal, and RViz running. Now you are good to navigate!

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/46efbdc6-3981-4021-aec3-2ff08b0d1ace)

As the robot navigates to the goal, you will see a lot of things printed on the terminal. These are just messages ROS is giving you as the navigation is performed. When it reaches the goal, you will see "Reached the goal!" in the terminal. 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/63dbc724-b983-451d-a50d-063ce97bf8fe)

## Behind the scenes: Navigation
The navigation stack is the most complicated part of our design project. There is a lot going on behind the scenes, which I will break down for your understanding. The list of things involved, in order, goes as follows:
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
