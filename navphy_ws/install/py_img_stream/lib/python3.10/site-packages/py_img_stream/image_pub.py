import time
import argparse
import cv2
from cv2 import aruco
import scipy.io as sio
# import pyrealsense2 as rs
import numpy as np
from imutils.video import FileVideoStream

# Import ROS specific packages
import rclpy
from rclpy.node import Node
import std_msgs.msg as std_m
from std_msgs.msg import String
from std_msgs.msg import Int32, Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import serial
import json
import time
from datetime import datetime
import threading
marker_information = Float32MultiArray()
class ImagePublisher(Node):

  def __init__(self):
    super().__init__('image_pub')
    self.marker_location_publisher = self.create_publisher(Float32MultiArray, "/marker_position", 1)
    self.target_distance_publisher = self.create_publisher(Float32, "/target_distance", 1)
    self.target_spotted_publisher = self.create_publisher(Int32, "/target_spotted", 1)

    timer_period = .1
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.get_logger().info('Initialized timer')

    self.camParams = sio.loadmat("./calibration/dopeCam_params.mat")
    self.cameraMatrix = self.camParams['cameraMatrix']
    self.distCoeffs = self.camParams['distortionCoefficients']

    self.angle = []
    self.marker_position = Float32MultiArray()
    self.translation = Float32MultiArray()
    self.target_distance = Float32()
    self.target_spotted = Int32()
    self.target_spotted.data = 0

    self.target_subscription = self.create_subscription(
      Int32, 
      '/target_id', 
      self.target_acquire,
      10)
    self.target_prev = 9999

    self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    self.aruco_params = aruco.DetectorParameters_create()
    self.resolutionX = 1920
    self.resolutionY = 1080
    self.target = 9999

    self.markerLength = 88.9 # mm
    self.marker_side = self.markerLength

    if self.cameraMatrix is not None:
      self.get_logger().info('Starting capture')
      
    self.cam = cv2.VideoCapture(0,cv2.CAP_V4L2)
    self.cam.set(cv2.CAP_PROP_MODE,0)
    # self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolutionX)
    # self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolutionY)
    self.cam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
    self.cam.set(cv2.CAP_PROP_FPS,60)
    self.cam.set(cv2.CAP_PROP_EXPOSURE,-20.0)
    self.cam.set(28,0)
    self.frame = []
    self.ret = False      
    # threadout2 = threading.Thread(target=self.cv2_show)
    # threadout2.start()
    # threadout2.join()
    self.marker_ids_seen = set()
    self.custom_marker_sides = dict()
    self.marker_pose = []
    self.recording_loc= f"/run/user/1000/gvfs/google-drive:host=yahoo.com,user=sdpteam12.2023/GVfsSharedWithMe/1vVJEmEOf5WBflh5dtDjcaZt9LaQs4gp1/RECORDING-{datetime.now()}.avi"
    self.isRecording = Int32()
    self.target_spotted_time = time.time()      
    self.output = None
    # threadout2 = threading.Thread(target=self.cv2_show)
    # threadout2.start()
    # threadout2.join()

    self.recordingpub = self.create_publisher(Int32, "/recording", 1)

    capture_thread = threading.Thread(target=self.cv2_capture)
    capture_thread.daemon = True
    capture_thread.start()

  def target_acquire(self,msg):
    print("target acquired")

    self.target = msg.data
    if self.target != self.target_prev:
      self.isRecording.data = 0
      self.target_prev = self.target

  def get_pose(self):
    corners, ids, rejected = aruco.detectMarkers(self.frame, self.aruco_dict, parameters = self.aruco_params)

    self.currently_seen_ids = set()      
    # threadout2 = threading.Thread(target=self.cv2_show)
    # threadout2.start()
    # threadout2.join()
    if ids is not None and len(ids) > 0: 

      ids_array = Int32MultiArray()
      for (marker_corner, marker_id) in zip(corners, ids):
        if(marker_id == self.target):
          self.currently_seen_ids.add(int(marker_id[0]))
          ids_array.data = self.currently_seen_ids
          marker_side = self.marker_side

          rvec, tvec, _ = aruco.estimatePoseSingleMarkers(marker_corner, marker_side, 
            self.cameraMatrix,self.distCoeffs)

          # stri = String()
          # stri.data = "\n ID: {0} \n T (X,Y,Z): {1} \n R:{2}".format(marker_id[0], tvec[0][0], rvec[0][0])
          self.translation.data = [float(tvec[0][0][0]),float(tvec[0][0][1]),float(tvec[0][0][2])]
          self.target_distance.data = float(tvec[0][0][2])
        #   self.translation_pubMJPGlisher.publish(self.translation)
    return (corners,ids)
  
  def get_pixel_pos(self,corners,ids):
    for (markerCorner, markerID) in zip(corners, ids):
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        self.marker_position.data = [float(cX-(self.resolutionX//2)),float(cY-(self.resolutionY/2))]

  def cv2_capture(self):
    while True:
      self.ret, self.frame = self.cam.read()

      if self.target_spotted.data and (self.target_distance.data / 1000.0) <= 3.0:
        self.isRecording.data = 1
        self.target_spotted_time = time.time()
        self.output = cv2.VideoWriter(self.recording_loc,cv2.VideoWriter_fourcc(*"XVID"),60,(1280,720))

      if(time.time()-self.target_spotted_time) > 5 and self.isRecording.data:
        # self.get_logger().info('Stopping recording')
        self.isRecording.data = 0

      if self.isRecording.data:
        # self.get_logger().info('Recording...')
        threadout = threading.Thread(target=self.cv2_record)
        threadout.start()
        threadout.join()
      elif self.output:
        print("releasing")
        self.output.release()

      # threadout2 = threading.Thread(target=self.cv2_show)
      # threadout2.start()
      # threadout2.join()

  def cv2_record(self):
    self.output.write(self.frame)

  def cv2_show(self):
    pass
  
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
        # convert each of the (x, y)-co          # print(self.angle,"\n", tvec,"\n",rvec)ordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
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
    self.target_spotted.data = 0

    if self.ret == True:
      (corners,ids) = self.get_pose()
      # self.get_logger().info("getting stuff")
      # cv2.imshow('camera', self.frame)
      if(corners is not None and ids is not None):
        self.aruco_display(corners,ids)
        self.get_pixel_pos(corners,ids)

        for (markerCorner, markerID) in zip(corners, ids):
          if(markerID == self.target):         
            # self.get_logger().info("%s" % json.dumps(list(self.marker_position.data) + [self.translation.data[2]]))
            global marker_information
            marker_information.data =[self.marker_position.data[0],self.marker_position.data[1]]

            self.marker_location_publisher.publish(marker_information)
            self.target_distance_publisher.publish(self.target_distance) 
            self.target_spotted.data = 1 
          else:
            self.target_spotted.data = 0

      # For recording
    self.target_spotted_publisher.publish(self.target_spotted)
    self.recordingpub.publish(self.isRecording)

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