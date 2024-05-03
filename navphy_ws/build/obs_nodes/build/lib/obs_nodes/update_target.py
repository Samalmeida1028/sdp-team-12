from obswebsocket import obsws, requests


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray




class OBSTargetUpdater(Node):


    def __init__(self):
        super().__init__('obs_target_update')
        self.host = "192.168.137.122"
        self.port = 4455
        self.password = "MhusmcBKXLoWnJOX"  # Replace with your OBS WebSocket password
        self.ws = obsws(self.host, self.port, self.password)
        self.ws.connect()
        self.ws.call(requests.SetInputSettings(
        inputName='target_id',
        inputSettings={"text": 'Acquiring target.'}
        ))

        self.target_subscription = self.create_subscription(
        Int32, 
        '/target_id', 
        self.target_acquire,
        10)

    def target_acquire(self,msg):
        self.ws.call(requests.SetInputSettings(
        inputName='target_id',
        inputSettings={"text": f"current target is: {msg.data}"}
        ))
        



def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = OBSTargetUpdater()
  
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