'''
Authors: Arjun Viswanathan, Samuel Almeida
Date last modified: 4/26/24
Target Publisher:
- Gives equal amount of time before publishing new targets
- Targets can be read in from a .txt file
- .txt file can be updated without killing the node
- Wait until within range of target and seeing target in camera frame to tick timer
'''

# Import ROS specific packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32,Float32
import json
import os

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_pub')

        self.publisher = self.create_publisher(Int32, "/target_id", 10)
        self.recording_time_publisher = self.create_publisher(Float32,"/recording_time",10)
        self.recording_max_time_publisher = self.create_publisher(Float32,"/recording_max_time",10)
        self.marker_side_pub = self.create_publisher(Int32, "/marker_side", 10)

        self.recordingsub = self.create_subscription(Int32, "/recording", self.set_recording, 10)
        self.spottedsub = self.create_subscription(Int32, "/target_spotted", self.set_spotted, 10)

        self.targets = []
        self.file_index = 0
        self.numtargets = 0
        self.spotted = 0
        self.is_recording = 0
        self.publish_dummy_once = True
        self.start_padding_counter = False

        self.target_id = Int32()
        self.recording_time = Float32()
        self.recording_time.data = 0.0
        self.recording_max_time = Float32()
        self.recording_max_time.data = -1.0
        self.marker_side = Int32()

        self.fileupdater = self.create_timer(1, self.update)
        self.cycle = self.create_timer(0.5, self.cycle_targets)

        self.get_logger().info('Target Publisher node ready!')

    def update(self): 
        # constantly check file updates for new targets
        if os.stat("targets.txt").st_size != 0:
            with open("targets.txt") as f:
                try: self.targets = json.load(f)
                except json.decoder.JSONDecodeError as E:
                    self.get_logger().info("File is incorrectly formatted")
        else:
            self.targets = []

        # print(self.targets)
        self.numtargets = len(self.targets)
        
        # Clearing targets
        if self.numtargets < self.file_index: # to reset the list of targets
            self.file_index = 0
            # self.padding_time = 5.0
            self.recording_max_time.data = -1.0
            self.recording_time.data = 0.0
            self.target_id.data = 9999
            self.get_logger().info('Targets cleared. Resetting...')

        if self.target_id.data == 9999:
            self.recording_max_time.data = -1.0
            self.recording_time.data = 0.0
        else:
            # Publishing
            self.marker_side_pub.publish(self.marker_side)

        self.publisher.publish(self.target_id)
        self.recording_time_publisher.publish(self.recording_time)
        self.recording_max_time_publisher.publish(self.recording_max_time)

    def set_recording(self, msg : Int32):
        self.is_recording = msg.data

    def set_spotted(self, spottedmsg: Int32):
        self.spotted = spottedmsg.data

    def set_target(self):
        self.file_index += 1
        self.recording_time.data = 0.0

    def is_eof(self):
        return self.file_index == self.numtargets
    
    def cycle_targets(self):
        if self.recording_time.data >= self.recording_max_time.data and not self.is_eof(): # after timeout, then change target
            # self.start_padding_counter = True
            self.target_id.data = int(self.targets[self.file_index][0])
            self.marker_side.data = int(self.targets[self.file_index][1])
            self.recording_max_time.data = float(self.targets[self.file_index][2])

            self.get_logger().info('Got new target {} with time {} seconds'.format(self.target_id.data, self.recording_max_time.data))
            self.set_target()
        elif self.recording_time.data < self.recording_max_time.data:
            if self.is_recording: # for recording or not
                # self.get_logger().info('Recording target {} for {} seconds'.format(self.target_id.data, self.recording_time))
                self.recording_time.data += 0.5
        elif self.is_eof():
            self.file_index = 0
            # self.get_logger().warn('Reached EOF. Cycling to start of targets')

def main():
    rclpy.init()
    node = TargetPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()