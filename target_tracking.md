# Current Working Tracking:

**Currently, the tracking works via a few steps:**

1. Detection of arUco markers with the OpenCV arUco detection library
2. Passing the coordinate information via serial to a microcontroller
3. The controller uses the coordinates as an offset from the center of the image, and that offset is the error between the actual position and desired position
4. The error is fed into a closed-loop control system, utilizing a PID controller
5. The output is converted to x-y angles for two servo motors
