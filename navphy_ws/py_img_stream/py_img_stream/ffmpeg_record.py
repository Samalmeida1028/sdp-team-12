# Authors: Arjun Viswanathan, Samuel Almeida
# Date created: 4/25/24
# Date last modified: 4/25/24

import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import Int32
from datetime import datetime  
import threading

class FFMPeg_Record(Node):
    def __init__(self):
        super().__init__('ffmpeg_record')

        self.recordingsub = self.create_subscription(Int32, "/recording", self.set_recording, 10)
        self.recordingtimer = self.create_timer(0.0333, self.record)

        self.ffmpeg_cmd = [
        'ffmpeg',
        '-f', 'v4l2', '-i', '/dev/video3',   # Video input from /dev/video0
        '-f', 'alsa', '-i', 'default',       # Audio input from default ALSA device
        '-c:v', 'copy',                      # Copy video stream without re-encoding
        '-c:a', 'aac',                       # Encode audio stream using AAC codec
        '-f', 'mpegts',
        '-'       # Output filename
        ]
        self.recording_pid = subprocess.Popen(self.ffmpeg_cmd, stdout=subprocess.PIPE)
        self.recording_path = "recordings/outputs/"

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path_to_output = self.recording_path + f"output_{timestamp}.avi"
        self.output = open(self.path_to_output, 'wb')

        self.frame = None
        self.is_recording = 0

        read_thread = threading.Thread(target=self.read)
        read_thread.daemon = True
        read_thread.start()

        self.get_logger().info("FFMPeg Record node ready!")

    def set_recording(self, rmsg : Int32):
        self.is_recording = rmsg.data

    def read(self):
        self.get_logger().info("Starting read frames from ffmpeg stdout")
        while True:
            self.frame = self.recording_pid.communicate()

    def record(self):
        if self.is_recording:
            self.output.write(self.frame)

def main(args=None):
    rclpy.init(args=args)
    ffmpeg_record = FFMPeg_Record()
    rclpy.spin(ffmpeg_record)
    ffmpeg_record.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()