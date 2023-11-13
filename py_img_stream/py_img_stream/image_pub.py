import time
import argparse
import cv2
from cv2 import aruco
import scipy.io as sio
# import pyrealsense2 as rs
import numpy as np

# Import ROS specific packages
import rclpy
from rclpy.node import Node
import std_msgs.msg as std_m
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import serial
import json


class ImagePublisher(Node):

  def __init__(self):
    super().__init__('image_pub')
    self.ser = serial.Serial(
             '/dev/ttyACM1',
             baudrate=115200,
             timeout=0.01)
    
    self.translation_publisher = self.create_publisher(Float32MultiArray, "translation_list", 1)
    self.rotation_publisher = self.create_publisher(Float32MultiArray, "rotation_list", 1)
    self.position_publisher = self.create_publisher(Float32MultiArray, "xyPos", 1)
    timer_period = .016
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.get_logger().info('Initialized timer')
    self.camParams = sio.loadmat("./calibration/logi_camParams.mat")
    self.cameraMatrix = self.camParams['cameraMatrix']
    self.distCoeffs = self.camParams['distortionCoefficients']
    self.angle = 0
    self.marker_position = Float32MultiArray()
    self.translation = Float32MultiArray()

    self.target_subscription = self.create_subscription(
      Int32, 
      'target_id', 
      self.target_acquire,
      10)

    self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    self.aruco_params = aruco.DetectorParameters_create()
    self.resolutionX = 1920
    self.resolutionY = 1080
    self.target = 9999
    # self.aruco_params = cv2.arbytearray(datastring,encoding="utf-8")uco.DetectorParameters()
    # self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
    self.markerLength = 60 # mm
    self.marker_side = self.markerLength
    if self.cameraMatrix is not None:
      self.get_logger().info('Starting capture')
    self.cam = cv2.VideoCapture(2,cv2.CAP_V4L2)
    self.cam.set(cv2.CAP_PROP_MODE,0)
    self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolutionX)
    self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolutionY)
    self.cam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
    self.cam.set(cv2.CAP_PROP_FPS,30)
    self.frame = []
    self.marker_ids_seen = set()
    self.custom_marker_sides = dict()
    self.marker_pose = []
    # self.br = CvBridge()

  def target_acquire(self,msg):
    self.target = msg.data
    # self.get_logger().info("%d" %msg.data)

  def get_pose(self):
    corners, ids, rejected = aruco.detectMarkers(self.frame, self.aruco_dict, parameters = self.aruco_params)

    # self.get_logger().warn("ids: {0}".format(ids))
    self.currently_seen_ids = set()
    
    # self.get_logger().info("here")
    if ids is not None and len(ids) > 0: 
      # self.aruco_display(corners,ids)
      # self.get_logger().info("now here")

      # if self.publish_image_feedback:
      #   self.aruco_display(corners, ids)
      ids_array = Int32MultiArray()
      translations = Float32MultiArray()
      rotations = Float32MultiArray()
      for (marker_corner, marker_id) in zip(corners, ids):
        if(marker_id == self.target):
          self.currently_seen_ids.add(int(marker_id[0]))
          ids_array.data = self.currently_seen_ids
          marker_side = self.marker_side


          rvec, tvec, _ = aruco.estimatePoseSingleMarkers(marker_corner, marker_side, 
            self.cameraMatrix,self.distCoeffs)
          # translations.data = np.array(tvec[0][0]).astype(float)
          # rotations.data = np.array(rvec[0][0]).astype(float)
          


          # if self.use_custom_marker_side_dict and marker_id[0] in self.custom_marker_sides:
          #   marker_side = self.custom_marker_sides[marker_id[0]]

          # Pose estimation for each marker

          # if not self.search_for_grid or marker_id[0] not in self.grid_overall_ids:
          self.get_logger().warn("\n ID: {0} \n T (X,Y,Z): {1} \n R:{2}".format(marker_id[0], tvec[0][0], rvec[0][0]))
          stri = String()
          stri.data = "\n ID: {0} \n T (X,Y,Z): {1} \n R:{2}".format(marker_id[0], tvec[0][0], rvec[0][0])
          self.translation.data = [float(tvec[0][0][0]),float(tvec[0][0][1]),float(tvec[0][0][2]), float(self.angle)]
          self.get_logger().info("%s" % str(self.translation.data))
          self.translation_publisher.publish(self.translation)
    return (corners,ids)
  



  def get_pixel_pos(self,corners,ids):
    for (markerCorner, markerID) in zip(corners, ids):

        # Draw the axis on the aruco markers        print(tvecoord)

        # extract the marker corners (which are always returned in
        # top-left, top-right, bottom-right, and bottom-left order)
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        self.marker_position.data = [float(cX-(self.resolutionX//2)),float(cY-(self.resolutionY/2))]
        if(markerID == int(self.target)):
          self.get_logger().info("target stuff")
              

  def aruco_display(self, corners, ids):
    if len(corners) > 0:
        # flatten the ArUco IDs list
      ids = ids.flatten()
      # self.frame_color = cv2.cvtColor(self.frame, cv2.COLOR_GRAY2BGR)
      # loop over the detected ArUCo corners
      for (markerCorner, markerID) in zip(corners, ids):

        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(markerCorner, self.marker_side, 
            self.cameraMatrix, self.distCoeffs)
        # Draw the axis on the aruco markers
        aruco.drawAxis(self.frame, self.cameraMatrix, self.distCoeffs, rvec, tvec, 0.1)

        # extract the marker corners (which are always returned in
        # top-left, top-right, bottom-right, and bottom-left order)
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
        tvecoord = (int(tvec[0][0][0]),int(tvec[0][0][1]))
        cv2.line(self.frame, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(self.frame, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(self.frame, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(self.frame, bottomLeft, topLeft, (0, 255, 0), 2)

        # compute and draw the center (x, y)-coordinates of the ArUco
        # marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(self.frame, (cX, cY), 4, (0, 0, 255), -1)
        # draw the ArUco marker ID on the image          # self.publisher.publish(stri)
        cv2.putText(self.frame, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)

   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.016 seconds.
    """
    ret, self.frame = self.cam.read()
    if ret == True:
      (corners,ids) = self.get_pose()
      # self.get_logger().info("getting stuff")
      # cv2.imshow('camera', self.frame)
      if(corners is not None and ids is not None):
        self.get_pixel_pos(corners,ids)
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
        self.aruco_display(corners,ids)
        self.ser.write(bytearray(json.dumps(list(self.marker_position.data) + [self.translation.data[2]]) + "\n",encoding="utf-8"))
      serial_in = self.ser.readline()
      if serial_in:
        self.angle = int(json.loads(serial_in.decode('utf-8')))
      cv2.imshow('camera',self.frame)
      cv2.waitKey(16)

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