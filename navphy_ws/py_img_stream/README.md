# ArUco Detection and Pose Estimation
Below is our documentation on how we perform our computer vision for marker detection. Firstly, we calibrate the camera using 10 images of a checkerboard that we took, with different distances and angles. Using this, we run a [```MATLAB calibration script```](https://github.com/arjuns-code-center/G.L.I.M.P.S.E/blob/main/navphy_ws/calibration/camera_calibrator_script.m) to get our camera matrix parameters and distortion coefficients. This saves a parameters.mat file in our [```calibration```](https://github.com/arjuns-code-center/G.L.I.M.P.S.E/tree/main/navphy_ws/calibration) directory. We then use this .mat file in our code to perform detection and pose estimation. 

# The Translational Vector
We are interested in the translational vector for our project. It is as follows:

``tvec = [x, y, d]`` where

``x``: vertical position of the center of the marker with respect to the center of the frame

``y``: horizontal position of the center of the marker with respect to the center of the frame

``d``: distance to the center of the marker from the camera.

# Output

Here is how the detected image looks like. This information gets passed down to our serial node which will move the servos to the center of the marker. 

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/31181203-7a5d-4ea5-ab4f-5fbfafec20a9)

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/b56cac42-61f8-4909-b5ae-4cb6107bcbe3)

Here is what the console output is.

![image](https://github.com/Samalmeida1028/sdp-team-12/assets/41523488/87d7b692-d8ad-4f8e-a7d1-e9cf463e3aaa)
