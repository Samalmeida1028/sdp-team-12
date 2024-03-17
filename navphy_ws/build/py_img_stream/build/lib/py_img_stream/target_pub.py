import numpy as np
import time

# Import ROS specific packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32

# TODO: pause timer when target not seen but in range still in cycle_targets()
class TargetPublisher(Node):

    def __init__(self):
        super().__init__('target_pub')
        self.publisher = self.create_publisher(Int32, "/target_id", 10)

        self.spotted = self.create_subscription(Int32, "/target_spotted", self.cycle_targets, 10)
        self.target_distance = self.create_subscription(Float32, "/target_distance", self.set_distance, 10)

        self.targets = []
        self.index = 0
        self.d = 9999
        self.numtargets = 0

        self.target_id = Int32()
        self.record_timeout = 10
        self.record_start_time = time.time()
        self.recording_time = 0

        self.txtfileupdater = self.create_timer(1, self.update_txt)

        self.get_logger().info('Target Publisher node ready!')

    def set_distance(self, msg: Float32):
        self.d = msg.data / 1000.0

    def update_txt(self): # constantly check file updates for new targets
        with open("targets.txt") as f:
            self.targets = [int(x) for x in f.read().split()]
        self.numtargets = len(self.targets)

    def set_target(self):
        for i in range(10):
            self.publisher.publish(self.target_id)

    def cycle_targets(self, msg: Int32):
        if self.index == 0 and self.index < self.numtargets: # send first target if it is in file
            self.target_id.data = int(self.targets[self.index])
            self.get_logger().info('Got initial target {}'.format(self.target_id.data))
            self.set_target()
            self.index += 1
        elif self.index < self.numtargets: # give every subsequent target from file equal time in recording
            if msg.data and self.d <= 3.0: # wait until within steady state distance and target is spotted
                time_now = time.time()
                self.recording_time = time_now - self.record_start_time
                self.get_logger().info('Time recorded {}'.format(self.recording_time))

                if self.recording_time >= self.record_timeout: # after timeout, then change target
                    self.target_id.data = int(self.targets[self.index])
                    self.get_logger().info('Got new target {}'.format(self.target_id.data))
                    self.set_target()
                    self.index += 1
                    self.d = 9999
                    self.record_start_time = time.time() # reset timer
            elif not msg.data and self.d <= 3.0:
                self.record_start_time = time.time() - self.recording_time # pause timer
                self.get_logger().info('Paused timer at {}'.format(self.record_start_time))
            else:
                self.record_start_time = time.time() # reset timer
                self.get_logger().info('Reset timer to {}'.format(self.record_start_time))

def main():
    rclpy.init()
    node = TargetPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()