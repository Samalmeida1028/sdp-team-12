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
    self.publisher = self.create_publisher(String, "test", 10)
    timer_period = .016
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.get_logger().info('Initialized timer')
    self.camParams = sio.loadmat("./calibration/sam_camParams.mat")
    self.cameraMatrix = self.camParams['cameraMatrix']
    self.distCoeffs = self.camParams['distortionCoefficients']

    self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    self.aruco_params = aruco.DetectorParameters_create()
    # self.aruco_params = cv2.arbytearray(datastring,encoding="utf-8")uco.DetectorParameters()
    # self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
    self.markerLength = 130 # mm
    self.marker_side = self.markerLength
    if self.cameraMatrix is not None:
      self.get_logger().info('Starting capture')
    self.cam = cv2.VideoCapture(2) 
    self.cam.set(cv2.CAP_PROP_MODE,0)
    self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    self.cam.set(cv2.CAP_PROP_FPS,30)
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
      # self.aruco_display(corners,ids)
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
        translation = Float32MultiArray()
        translation.data = [float(tvec[0][0][0]),float(tvec[0][0][1]),float(tvec[0][0][2])]
        # self.publisher.publish(stri)
        # self.translation_publisher.publish(translation)
    return (corners,ids)
  



  def get_pixel_pos(self,corners,ids):
    marker_position = Float32MultiArray()
    for (markerCorner, markerID) in zip(corners, ids):

        # Draw the axis on the aruco markers        print(tvecoord)

        # extract the marker corners (which are always returned in
        # top-left, top-right, bottom-right, and bottom-left order)
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        marker_position.data = [float(cX-320),float(cY-240)]

        self.translation_publisher.publish(marker_position)
    return marker_position
              
      
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
      # self.presence_pub[marker_not_seen].publish(presence_msg)

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
        # draw the ArUco marker ID on the image
        cv2.putText(self.frame, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)

   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.02 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, self.frame = self.cam.read()
    # img = cv2.flip(img, -1)
    # self.frame = cv2.flip(img, 1)


    # self.get_logger().info('Reading camera')
    if ret == True:
    #   # Publish the image.
    #   # The 'cv2_to_imgmsg' method converts an OpenCV
    #   # image to a ROS 2 image message
      # pose = str(self.get_pose())
      # test = String()
      # test.data = poseframe_colo
      # if(pose is not None):
      #   self.publisher.publish(test)
      # self.get_logger().info(type(img))
      (corners,ids) = self.get_pose()
      # cv2.imshow('camera', self.frame)
      if(corners is not None and ids is not None):
        self.get_pixel_pos(corners,ids)
        self.aruco_display(corners,ids)
      cv2.imshow('camera',self.frame)
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