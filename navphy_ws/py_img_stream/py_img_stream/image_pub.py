import time
import cv2
from cv2 import aruco
import scipy.io as sio
import time

# Import ROS specific packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

marker_information = Float32MultiArray()
capture_threads = []
class ImagePublisher(Node):
  def __init__(self):
    super().__init__('img_pub')

    self.marker_location_publisher = self.create_publisher(Float32MultiArray, "/marker_position", 1)
    self.target_distance_publisher = self.create_publisher(Float32, "/target_distance", 1)
    self.target_spotted_publisher = self.create_publisher(Int32, "/target_spotted", 1)

    self.markersidesub = self.create_subscription(Int32, "/marker_side", self.set_marker_length, 10)

    timer_period = .03 # 2 Hz = 20% CPU, 10 Hz = 30% CPU, 20 Hz = 90+% CPU (nav does not work above 70% CPU usage as well)
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

    self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    self.aruco_params = aruco.DetectorParameters_create()
    self.aruco_params.errorCorrectionRate = 1.0
    self.aruco_params.cornerRefinementMaxIterations = 5
    self.resolutionX = 1920
    self.resolutionY = 1080 # for recording, this is the only resolution that gives 60 fps outputs 
    self.target = 9999
    self.prev_target = 9999

    self.marker_length = 100 # mm

    self.marker_ids_seen = set()
    self.custom_marker_sides = dict()
    self.marker_pose = []
    self.count = 0

    if self.cameraMatrix is not None:
      self.get_logger().info('Starting capture')

    self.cam = cv2.VideoCapture(2,cv2.CAP_V4L2)
    self.cam.set(cv2.CAP_PROP_MODE,0)
    self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolutionX)
    self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolutionY)
    self.cam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
    self.cam.set(cv2.CAP_PROP_FPS,30)
    self.cam.set(cv2.CAP_PROP_EXPOSURE,-20.0)
    self.cam.set(28,0)
    self.frame = []
    self.ret = False

    self.get_logger().info('Created video capture at {} FPS'.format(self.cam.get(cv2.CAP_PROP_FPS)))

    self.recording_path = "recordings/"

    self.isRecording = Int32()
    self.target_spotted_time = time.time()

    self.recordingpub = self.create_publisher(Int32, "/recording", 1)

  def target_acquire(self,msg):
    # self.get_logger().info("target acquired")
    self.target = msg.data
    # print(self.target)

  def set_marker_length(self, msg : Int32):
    self.marker_length = msg.data
    # self.get_logger().info("Got new marker length {}".format(self.marker_length))

  def get_pose(self):
    self.ret, self.frame = self.cam.read()
    # img_gray = cv2.cvtColor(src=self.frame, code=cv2.COLOR_RGB2GRAY)
    got_marker = False
    if self.ret:
      # self.get_logger().info("Ripping")
      corners, ids, rejected = aruco.detectMarkers(self.frame, self.aruco_dict, parameters = self.aruco_params)
      if ids is not None and len(ids) > 0: 

        # ids_array = Int32MultiArray()
        for (marker_corner, marker_id) in zip(corners, ids):
          if(marker_id == self.target):
            marker_side = self.marker_length
            # print(marker_length)
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(marker_corner, marker_side, 
            self.cameraMatrix,self.distCoeffs)
            corners = marker_corner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            self.marker_position.data = [float(cX-(self.resolutionX//2)),float(cY-(self.resolutionY/2))]
            self.translation.data = [float(tvec[0][0][0]),float(tvec[0][0][1]),float(tvec[0][0][2])]
            self.target_distance.data = float(tvec[0][0][2])
            got_marker = True
      return got_marker

  def capture(self):
    if self.target == self.prev_target and self.target != 9999:
      if self.target_spotted.data and (self.target_distance.data / 1000.0) <= 4.0:
        self.isRecording.data = 1
        self.target_spotted_time = time.time()

    if (time.time()-self.target_spotted_time) > 5 and self.isRecording.data:
      self.isRecording.data = 0

    if self.target != self.prev_target:
      self.prev_target = self.target
      self.isRecording.data = 0
        
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.05 seconds = 20 Hz
    """   
    marker = self.get_pose()
    # self.get_logger().info("getting stuff")
    # cv2.imshow('camera', self.frame).markerborder
    
    if(marker):
      global marker_information
      marker_information.data =[self.marker_position.data[0],self.marker_position.data[1]]

      self.marker_location_publisher.publish(marker_information)
      self.target_distance_publisher.publish(self.target_distance) 
      self.target_spotted.data = 1 
    else:
      self.target_spotted.data = 0

    self.capture()
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