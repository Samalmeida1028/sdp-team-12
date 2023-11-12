import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import std_msgs.msg
import serial
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
import json


class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_sub')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    # self.subscription = self.create_subscription(
    #   String,     wait_set.wait(timeout_nsec
    #   'test', 
    #   self.listener_callback, 
    #   10)
    # self.subscription # prevent unused variable warning
    self.translation_subscription = self.create_subscription(
      Float32MultiArray, 
      'xyPos', 
      self.listener_callback, 
      10)
    self.translation_subscription # prevent unused variable warning
    self.ser = serial.Serial(
             '/dev/ttyACM2',
             baudrate=115200,
             timeout=0.01)
    self.angle_publisher = self.create_publisher(Int32, "angle", 1)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, dataa):
    """
    Callback function.
    """
    # Display the message on the console
 
    # Convert ROS Image message to OpenCV image
    # self.get_logger().info('I heard image')
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # current_frame = self.br.imgmsg_to_cv2(gray)
    
    # # Display image
    # cv2.imshow("camera", current_frame)
    datastring = str(str(dataa.data[0]) + " " + str(dataa.data[1]))
    # self.get_logger().info("%s" % datastring)
    self.ser.write(bytearray(json.dumps(list(dataa.data)) + "\n",encoding="utf-8"))
    serial_in = self.ser.readline()
    if serial_in:
      self.angle = Int32()
      self.angle.data = int(json.loads(serial_in.decode('utf-8')))
      self.angle_publisher.publish(self.angle)
      
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  "\n"
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()