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
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray


class ImagePublisher(Node):

  def __init__(self):
    super().__init__('image_pub')
    self.translation_publisher = self.create_publisher(Float32MultiArray, "translation_list", 1)
    self.rotation_publisher = self.create_publisher(Float32MultiArray, "rotation_list", 1)
    self.publisher_ids = self.create_publisher(Int32MultiArray, "ids_list", 10)
    self.publisher = self.create_publisher(String, "test", 10)
    timer_period = .016
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.get_logger().info('Initialized timer')
    self.camParams = sio.loadmat("./calibration/sam_laptop.mat")
    self.cameraMatrix = self.camParams['cameraMatrix']
    self.distCoeffs = self.camParams['distortionCoefficients']

    self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    self.aruco_params = aruco.DetectorParameters_create()
    # self.aruco_params = cv2.aruco.DetectorParameters()
    # self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
    self.markerLength = 51 # mm
    self.marker_side = self.markerLength
    if self.cameraMatrix is not None:
      self.get_logger().info('Starting capture')
    self.cam = cv2.VideoCapture(0) 
    self.frame = []
    self.marker_ids_seen = set()
    self.custom_marker_sides = dict()
    self.marker_pose = []
    # self.br = CvBridge()

  def get_pose(self):
    target_marker = 1 
    corners, ids, rejected = aruco.detectMarkers(self.frame, self.aruco_dict, parameters = self.aruco_params)

    # self.get_logger().warn("ids: {0}".format(ids))
    self.currently_seen_ids = set()
    
    # self.get_logger().info("here")
    if ids is not None and len(ids) > 0: 
      # self.get_logger().info("now here")

      # if self.publish_image_feedback:
      #   self.aruco_display(corners, ids)
      ids_array = Int32MultiArray()
      translations = Float32MultiArray()
      rotations = Float32MultiArray()
      for (marker_corner, marker_id) in zip(corners, ids):
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
        self.publisher.publish(stri)

        #   pass

              
      
      # if self.search_for_grid:

      #   for i in range (0, self.grid_number):
      #     retval, rvec2, tvec2 = aruco.estimatePoseBoard(corners, ids, self.boards[i], self.cam_params["mtx"], self.cam_params["dist"], rvec, tvec)

      #     self.get_logger().info("[Aruco Pose Estimator] retval: {0}".format(retval))
      #     if retval > 5:
      #       self.currently_seen_ids.add(self.grid_output_ids[i])
            
      #       if tvec2.shape[0] == 3:
      #         tvec2_ = [tvec2[0][0], tvec2[1][0], tvec2[2][0]]
      #         rvec2_ = [rvec2[0][0], rvec2[1][0], rvec2[2][0]]
            #   self.publish_pose(self.grid_output_ids[i], tvec2_, rvec2_)
            # else:
            #   self.publish_pose(self.grid_output_ids[i], tvec2[0][0], rvec2[0][0])


    for marker_not_seen in self.marker_ids_seen.difference(self.currently_seen_ids):
      presence_msg = Bool()
      presence_msg.data = False
      # self.presence_pub[marker_not_seen].publish(presence_msg)

   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.02 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, self.frame = self.cam.read()


    # self.get_logger().info('Reading camera')
    if ret == True:
    #   # Publish the image.
    #   # The 'cv2_to_imgmsg' method converts an OpenCV
    #   # image to a ROS 2 image message
      # pose = str(self.get_pose())
      # test = String()
      # test.data = pose
      # if(pose is not None):
      #   self.publisher.publish(test)
      #   self.get_logger().info(test)
      self.get_pose()
      cv2.imshow('camera', self.frame)
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