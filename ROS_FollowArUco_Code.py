# SDP Team 12 ArUco marker follow code
# Date created: 9/30/23
# Last modified date: 10/2/23
# Summary: follows a desired ArUco marker in front of it using computer vision

# How to run from command line:
# rosrun PKGNAME ROS_FollowArUco_Code.py
# rosrun PKGNAME ROS_FollowArUco_Code.py --timer=<TIME>
# For integration with keyboard_teleop, nothing to be done

# Import system packages
import time
import argparse
import cv2
import scipy.io as sio

# Import ROS specific packages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class FollowMarker(Node):
    def __init__(self, robot, timer=True):
        super.__init__('follow_marker')

        self.start_time = time.time()
        print("Starting Follow ArUco Algorithm...")        

        # TODO: intialize robot if needed
        ...
        self.timer = timer

        # Load camera parameters from MATLAB
        # TODO: Update path to camera params
        self.path = "..."

        # TODO: Decide which camera params to import
        camParams = sio.loadmat(self.path + "/d455i_camParams.mat")
        # camParams = sio.loadmat(self.path + "/webcam_camParams.mat")
        self.cameraMatrix = camParams['cameraMatrix']
        self.distCoeffs = camParams['distortionCoefficients']

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        # TODO: decide marker length
        self.markerLength = ... # mm

        # TODO: decide movement params for robot
        self.moveBy = ...
        self.rotBy = ...
        self.v = ...
        self.a = ...
        self.timeout = 1

        # TODO: decide actional distances for marker (in mm)
        self.distance = ...
        self.ignore = ...

        self.j = 0

        # TODO: figure out subscriber or whether we are using cv2.VideoCapture(0)
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.takeAction)
        rclpy.spin()
        # OR
        self.rgb_sub = cv2.VideoCapture(0)

    def takeAction(self, rgb_img):
        if self.timer:
            if (time.time() - self.start_time) > 30:
                self.robot.stop()
                rclpy.shutdown("Ending autonomous mode...")
                
        xm = 0
        xr = 0
        mp = [0, 0]
        depth = 0

        # TODO: update cv2 imread based on how we set up camera (subscriber in ROS or opencv-python)
        try:
            cv2_rgbimg = CvBridge().imgmsg_to_cv2(rgb_img, 'bgr8')

            if cv2_rgbimg is not None:
                s = cv2_rgbimg.shape # (height, width, channels)

                (corners, ids, rejected) = self.detector.detectMarkers(cv2_rgbimg)

                if len(corners) > 0:
                    ids = ids.flatten()

                    # For every detected marker, we do pose estimation using its corners and find the rotational and translational vectors
                    for (markerCorner, markerID) in zip(corners, ids):
                        reshapedCorners = markerCorner.reshape((4, 2))
                        (tL, tR, bR, bL) = reshapedCorners
                        tL = [int(tL[0]), int(tL[1])]
                        tR = [int(tR[0]), int(tR[1])]
                        bR = [int(bR[0]), int(bR[1])]
                        bL = [int(bL[0]), int(bL[1])]

                        mp[0] = int((tL[0] + bR[0]) / 2)
                        mp[1] = int((tL[1] + bR[1]) / 2)
                        
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.markerLength, self.cameraMatrix, self.distCoeffs)
                        rvec = rvec[0][0]
                        tvec = tvec[0][0]

                        depth = round(tvec[2], 2) # mm

                        # Printing distance on the image
                        cv2.putText(cv2_rgbimg, str(depth), (tL[0], tL[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        print("Marker detected! ID: {}, Center (pix): {}, Distance (mm): {}".format(str(markerID), mp, depth))

                    # Press 's' key when detecting marker to save image. Only available when marker is detected
                    if cv2.waitKey(33) == ord('s'):
                        print("Taking ArUco pic {}...".format(j))
                        cv2.imwrite(self.path + "Images/aruco_image_{}.png".format(self.j), cv2_rgbimg)
                        self.j += 1

                    if depth > self.ignore:
                        if depth > self.distance:
                            xm = self.moveBy
                        elif depth < self.distance:
                            xm = -self.moveBy

                    if mp[0] < s[0] / 2:
                        xr = -self.rotBy
                    elif mp[0] > s[0] / 2:
                        xr = self.rotBy
            else:
                print("No marker detected :(")
        except CvBridgeError as e:
            rclpy.logwarn('CV Bridge Error: {0}'.format(e))

        if xm != 0:
            self.move_base(xm)
        
        if xr != 0:
            self.rotate_base(xr)
            
        time.sleep(0.1)

    # TODO: write motor control commands
    def move_base(self, x):
        # Use distance
        
    def rotate_base(self, theta):
        # Use distance
        

if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument('--timer', default=False, type=str, help='what avoidance algorithm to run')
    args, unknown = args.parse_known_args()
    timer = int(args.timer)

    rclpy.init()
    FollowMarker(timer)