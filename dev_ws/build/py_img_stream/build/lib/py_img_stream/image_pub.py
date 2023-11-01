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
from std_msgs.msg import String


class ImagePublisher(Node):

  def __init__(self):
    super().__init__('image_pub')
    self.publisher = self.create_publisher(String, "video_frames", 1)
    timer_period = .016
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.get_logger().info('Initialized timer')
    camParams = sio.loadmat("./calibration/arjunPC_camParams.mat")
    cameraMatrix = camParams['cameraMatrix']
    distCoeffs = camParams['distortionCoefficients']

    self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    self.aruco_params = cv2.aruco.DetectorParameters()
    self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
    self.markerLength = 100 # mm

    self.get_logger().info('Starting capture')
    self.cam = cv2.VideoCapture(0) 
    # self.br = CvBridge()

  def get_pose(self):
    target_marker = 1 

    mp = [0, 0]
    depth = 0

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

   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.02 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cam.read()


    # self.get_logger().info('Reading camera')
    if ret == True:
    #   # Publish the image.
    #   # The 'cv2_to_imgmsg' method converts an OpenCV
    #   # image to a ROS 2 image message
      coords = get_pose();
      test = String()
      test.data = "FPSTEST"
      self.publisher.publish(test)
      cv2.imshow('camera', frame)
      cv2.waitKey(1)

    # Display the message on the console
    # self.get_logger().info('Publishing video frame')
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()