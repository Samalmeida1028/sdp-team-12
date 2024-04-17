# Authors: Samuel Almeida, Arjun Viswanathan

# Import ROS specific packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import threading
import sys

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_teleop')
        self.control_publisher = self.create_publisher(Float32MultiArray,"/marker_position",1)
        self.cam_pan_sub = self.create_subscription(String, "/camera_pan", self.move_camera, 10)

        timer_period = .05
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.vector = [0,0]

        # decay_and_print = threading.Thread(target=self.keyboard_to_vector)
        # decay_and_print.daemon = True
        # decay_and_print.start()

        self.message = Float32MultiArray()

    def move_camera(self, cam_msg : String):
        # while True:
        #     control = sys.stdin.readline()
        #     if control == 'w\n':
        #         self.vector[1] -=.05*1920
        #     if control == 'a\n':
        #         self.vector[0] -=.05*1080
        #     if control == 's\n':
        #         self.vector[1] +=.05*1920
        #     if control == 'd\n':
        #         self.vector[0] +=.05*1080

        if cam_msg.data == "w":
            self.vector[1] -=.05*1920
        if cam_msg.data == "a":
            self.vector[0] -=.05*1080
        if cam_msg.data == "s":
            self.vector[1] +=.05*1920
        if cam_msg.data == "d":
            self.vector[0] +=.05*1080

    def timer_callback(self):
        self.vector[0] = self.vector[0]*.9
        self.vector[1] = self.vector[1]*.9
        self.vector[0] = round(self.vector[0],3)
        self.vector[1] = round(self.vector[1],3)
        self.message.data = self.vector
        self.control_publisher.publish(self.message)
        # print(self.vector)
        # Display the message on the console
        # self.get_logger().info('Publishing video frame')
    
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  camera_controller = CameraController()
  
  # Spin the node so the callback function is called.
  rclpy.spin(camera_controller)
  
  camera_controller.destroy_node()

  rclpy.shutdown()
  
if __name__ == '__main__':
  main()