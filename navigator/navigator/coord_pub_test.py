import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class CoordPub(Node):
    def __init__(self):
        super().__init__("coords_pub")

        self.publisher = self.create_publisher(Float32MultiArray, '/translation_list', 10)
        
        time_period = 0.5
        self.timer = self.create_timer(time_period, self.timer_callback)
        
    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [46.0, 51.0, 40.0, 60.0, 50.0]
        self.publisher.publish(msg)

        print("Publishing {}".format(msg.data))

def main(args=None):
    rclpy.init(args=args)

    cpub = CoordPub()

    rclpy.spin(cpub)

    cpub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()