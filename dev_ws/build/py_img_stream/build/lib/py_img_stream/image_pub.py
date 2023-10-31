import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

from std_msgs.msg import String


class ImagePublisher(Node):

  def __init__(self):
    super().__init__('image_pub')
    self.publisher = self.create_publisher(String, "video_frames",1)
    timer_period = .001
    self.timer = self.create_timer(timer_period, self.timer_callback)

    self.cam = cv2.VideoCapture(0) 
    # self.br = CvBridge()
   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.02 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cam.read()

          
    if ret == True:
    #   # Publish the image.
    #   # The 'cv2_to_imgmsg' method converts an OpenCV
    #   # image to a ROS 2 image message
      cv2.imshow('camera', frame)
      cv2.waitKey(1)
    test = String()
    test.data = "FPSTEST"
    self.publisher.publish(test)

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