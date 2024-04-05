'''
Authors: Arjun Viswanathan, Samuel Almeida
Date last modified: 4/5/24
Target Publisher:
- Gives equal amount of time before publishing new targets
- Targets can be read in from a .txt file
- .txt file can be updated without killing the node
- Wait until within range of target and seeing target in camera frame to tick timer
'''

import time

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
        self.recording_max_time_publisher = self.create_publisher(Int32,"/recording_max_time",10)

        self.recordingsub = self.create_subscription(Int32, "/recording", self.set_recording, 10)
        self.spottedsub = self.create_subscription(Int32, "/target_spotted", self.set_spotted, 10)

        self.targets = []
        self.file_index = 0
        self.numtargets = 0
        self.spotted = 0
        self.is_recording = 0
        self.init_target_set = False

        self.target_id = Int32()
        # self.record_timeout = 10.0
        self.declare_parameter('recording_timeout',10)
        self.record_start_time = time.time()
        self.recording_time = Float32()
        self.recording_time.data = 0.0

        self.fileupdater = self.create_timer(1, self.update)
        self.cycle = self.create_timer(0.5, self.cycle_targets)

        self.get_logger().info('Target Publisher node ready!')

    def set_recording(self, msg : Int32):
        self.is_recording = msg.data

    def update(self): # constantly check file updates for new targets
        if os.stat("targets.txt").st_size != 0:
            with open("targets.txt") as f:
                try: self.targets = json.load(f)
                except json.decoder.JSONDecodeError as E:
                    self.get_logger().info("File is incorrectly formatted")
        else:
            self.targets = []
        # print(self.targets)

        self.numtargets = len(self.targets)

        if self.numtargets < self.file_index: # to reset the list of targets
            self.file_index = 0
            self.recording_time.data = float(self.get_parameter('recording_timeout').get_parameter_value().integer_value)
            self.init_target_set = False
            self.get_logger().info('Targets cleared. Resetting...')
            
        my_param = self.get_parameter('recording_timeout')

        # print(my_param.get_parameter_value().integer_value)
        self.set_parameters([my_param])

        self.recording_time_publisher.publish(self.recording_time)
        temp = Int32()
        temp.data = self.get_parameter("recording_timeout").get_parameter_value().integer_value
        self.recording_max_time_publisher.publish(temp)

    def set_spotted(self, spottedmsg: Int32):
        self.spotted = spottedmsg.data

    def set_target(self):
        self.publisher.publish(self.target_id)

        if self.target_id.data != 9999:
            self.file_index += 1
            self.record_start_time = time.time() # reset timer
            self.recording_time.data = time.time() - self.record_start_time

    def can_publish(self):
        return self.file_index <= self.numtargets and self.numtargets != 0
    
    def is_init_target(self):
        return self.file_index == 0 and not self.init_target_set

    def cycle_targets(self):
        if self.can_publish(): # give every subsequent target from file equal time in recording
            if self.is_init_target(): # send first target if it is in file
                self.target_id.data = int(self.targets[self.file_index])
                self.get_logger().info('Got initial target {}'.format(self.target_id.data))
                self.set_target()
                self.init_target_set = True

            if self.recording_time.data >= self.get_parameter('recording_timeout').get_parameter_value().integer_value: # after timeout, then change target
                try:
                    self.target_id.data = int(self.targets[self.file_index])
                    self.get_logger().info('Got new target {}'.format(self.target_id.data))
                    self.set_target()
                except IndexError:
                    self.target_id.data = 9999
                    self.get_logger().warn('Reached EOF! Waiting for new targets...')
                    self.set_target()
            else:
                if self.is_recording and self.spotted: # for recording or not
                    self.recording_time.data = time.time() - self.record_start_time
                    # self.get_logger().info('Recording target {} for {} seconds'.format(self.target_id.data, self.recording_time))
                else:
                    self.record_start_time = time.time() - self.recording_time.data # pause timer
                    # self.get_logger().info('Paused recording time at {} seconds'.format(self.recording_time))

def main():
    rclpy.init()
    node = TargetPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()