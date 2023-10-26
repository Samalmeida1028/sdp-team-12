# SDP Team 12 ArUco marker detection code
# Date created: 10/25/23
# Last modified date: 10/25/23
# Summary: obtains a target and publishes [X, Y, d] information

# How to run from command line:
# rosrun marker_detection MarkerDetection.py --camtype="webcam"

# TODO: test everything out and bug fixes. 
# TODO: make a launch file :)
# TODO: make a main script to publish targets for this script to use and test
# TODO: decide topic names for subscriber and publisher

# Import system packages
import time
import argparse
import cv2
import scipy.io as sio
import pyrealsense2 as rs
import numpy as np

# Import ROS specific packages
import rclpy
from rclpy.node import Node
import std_msgs.msg as std_m

class MarkerDetection(Node):
    def __init__(self, camType='webcam'):
        super.__init__('detect_marker')

        self.camType = camType

        self.start_time = time.time()
        print("Starting ArUco Detection Algorithm...")        

        # Load camera parameters from MATLAB
        # TODO: Update path to camera params
        self.path = "YOURSYSTEMPATHTOREPO/sdp-team-12/marker_detection/src/calibration"

        if camType == 'webcam':
            camParams = sio.loadmat(self.path + "/webcam_camParams.mat")
            self.cam = cv2.VideoCapture(0)
        elif camType == 'realsense':
            camParams = sio.loadmat(self.path + "/d455i_camParams.mat")

            self.pipe = rs.pipeline()
            cfg = rs.config()
            w = 640
            h = 480
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, 30)
            cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, 30)
            self.pipe.start()

        self.cameraMatrix = camParams['cameraMatrix']
        self.distCoeffs = camParams['distortionCoefficients']

        # Initialize ArUco disctionary and detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # TODO: decide marker length
        self.markerLength = 100 # mm

        self.j = 0

        # Obtains info from a topic and calls takeAction
        # Publishes [X, Y, d] to another topic once PoseEstimation is done
        self.target_sub = self.create_subscription(std_m.String, '/target', self.takeAction)
        self.coord_pub = self.create_publisher(std_m.Float64MultiArray, '/coords', 10)
        rclpy.spin()

    def takeAction(self, msg):   
        target_marker = int(msg.data) 

        mp = [0, 0]
        depth = 0

        if self.camType == 'webcam':
            success, rgb_img = self.cam.read()
        elif self.camType == 'realsense':
            frame = self.pipe.wait_for_frames()
            rgb_frame = frame.get_color_frame()
            rgb_img = np.asanyarray(rgb_frame.get_data())

        if rgb_img is not None:
            s = rgb_img.shape # (height, width, channels)

            (corners, ids, rejected) = self.detector.detectMarkers(rgb_img)

            if len(corners) > 0:
                ids = ids.flatten()

                # For every detected marker, we do pose estimation using its corners and find the rotational and translational vectors
                for (markerCorner, markerID) in zip(corners, ids):
                    if markerID == target_marker: # only execute if we have target marker
                        reshapedCorners = markerCorner.reshape((4, 2))
                        (tL, tR, bR, bL) = reshapedCorners
                        tL = [int(tL[0]), int(tL[1])]
                        tR = [int(tR[0]), int(tR[1])]
                        bR = [int(bR[0]), int(bR[1])]
                        bL = [int(bL[0]), int(bL[1])]

                        # mp[0] = int((tL[0] + bR[0]) / 2)
                        # mp[1] = int((tL[1] + bR[1]) / 2)
                        
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.markerLength, self.cameraMatrix, self.distCoeffs)
                        rvec = rvec[0][0]
                        tvec = tvec[0][0]

                        mp[0] = tvec[0]
                        mp[1] = tvec[1]

                        depth = round(tvec[2], 2) # mm

                        # Printing distance on the image
                        cv2.putText(rgb_img, str(depth), (tL[0], tL[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        print("Marker {} detected! X: {}, Y: {}, Distance (mm): {}".format(str(markerID), mp[0], mp[1], depth))
                    else:
                        print("Marker {} detected!".format(str(markerID)))

                # Press 's' key when detecting marker to save image. Only available when marker is detected
                if cv2.waitKey(33) == ord('s'):
                    print("Taking ArUco pic {}...".format(j))
                    cv2.imwrite(self.path + "Images/aruco_image_{}.png".format(self.j), rgb_img)
                    self.j += 1

                # Create an array of translational vector data
                coords = std_m.Float64MultiArray()
                coords.data = [mp[0], mp[1], depth]
                self.coord_pub.publish(coords)
        else:
            print("No marker detected :(")
            
        time.sleep(0.1)

if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument('--camtype', default="webcam", type=str, help='what kind of camera?')
    args, unknown = args.parse_known_args()
    camtype = int(args.camtype)

    rclpy.init()
    MarkerDetection(camtype)