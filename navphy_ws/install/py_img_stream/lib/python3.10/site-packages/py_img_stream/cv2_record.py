# Authors: Arjun Viswanathan, Samuel Almeida
# Date created: 4/25/24
# Date last modified: 4/25/24

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import cv2
import sounddevice as sd
from datetime import datetime
import wave
import threading
import subprocess
import time

class CV2Record(Node):
    def __init__(self):
        super().__init__('cv2_record')

        ################## Variables ##################
        self.is_recording = 0
        self.current_target = 9999

        ################## ROS ##################
        self.recordingsub = self.create_subscription(Int32, "/recording", self.record, 10)
        self.targetsub = self.create_subscription(Int32, "/target_id", self.target, 10)
        self.capture_callback = self.create_timer(1/30.0, self.cv2_capture)
        self.time = time.time()

        ################## Video ##################
        self.cam = cv2.VideoCapture(3,cv2.CAP_V4L2)
        self.cam.set(cv2.CAP_PROP_MODE,0)
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.resolutionX = 1920
        self.resolutionY = 1080
        self.cam.set(cv2.CAP_PROP_FPS,30)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolutionX)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolutionY)
        self.cam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))

        ################## Audio ##################
        self.rate = 44100
        self.channels = 1
        self.format = 'int16'

        ################## Paths ##################s
        self.recording_path = "recordings/"
        self.path_to_video_file = None
        self.path_to_audio_file = None
        self.path_to_merged_file = None

        ################## Writers/Flags ##################
        self.output = None
        self.waveFile = None
        self.output_released = True
        self.stream = None
        self.stream_started = False
        self.count = 0

        t = threading.Thread(target=self.read)
        t.daemon = True
        t.start()

    def record(self, recordingmsg : Int32):
        self.is_recording = recordingmsg.data

    def target(self, targetmsg : Int32):
        self.current_target = targetmsg.data

    def read(self):
        while True:
            self.ret, self.frame = self.cam.read()

    def cv2_capture(self):
        if self.is_recording: # is recording
            if self.output_released: # one time creation of writers for video and audio
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.path_to_video_file = self.recording_path + f"video_{timestamp}.avi"
                self.path_to_audio_file = self.recording_path + f"audio_{timestamp}.wav"
                self.path_to_merged_file = self.recording_path + f"output_{timestamp}.avi"

                self.output = cv2.VideoWriter(self.path_to_video_file,cv2.VideoWriter_fourcc(*"XVID"),30,(self.resolutionX,self.resolutionY))
                self.get_logger().info("Creating video writer at 30 FPS")

                self.waveFile = wave.open(self.path_to_audio_file, 'wb')
                self.waveFile.setnchannels(self.channels)
                self.waveFile.setsampwidth(2)
                self.waveFile.setframerate(self.rate)

                self.output_released = False
            
            if not self.stream_started: # start audio recording
                self.get_logger().info('Starting audio recording')
                self.stream = sd.InputStream(samplerate=self.rate, channels=self.channels, dtype=self.format, callback=self.write_to_wav)
                t1 = threading.Thread(target=self.start_stream)
                t1.start()
        elif self.stream_started: # not recording
            if self.current_target == 9999: # we have reached the end, so terminate
                self.get_logger().info('Stopping audio recording')
                self.stream.stop()
                self.stream_started = False
                self.stream.close()

                self.get_logger().info('Releasing writers for audio and video')
                self.output.release()
                self.waveFile.close()
                self.output_released = True

                self.get_logger().info('Quick merging video and audio')
                cmd = "ffmpeg -ac 1 -i " + self.path_to_video_file + " -i " + self.path_to_audio_file + " -c:v copy -c:a aac -strict experimental " + self.path_to_merged_file
                subprocess.call(cmd, shell=True)

                self.get_logger().info('Copying merged file into new directory')
                mv_cmd = "cp " + self.path_to_merged_file + " recordings/outputs/"
                subprocess.Popen(mv_cmd.split(' '))
            else: # we have moved to a new target, so pause
                self.get_logger().info('Pausing audio recording')
                t2 = threading.Thread(target=self.stop_stream)
                t2.start()

        if not self.output_released and self.stream_started:
            self.output.write(self.frame)
    
    def start_stream(self):
        self.stream.start()
        self.stream_started = True

    def stop_stream(self):
        self.stream.stop()
        self.stream_started = False
        self.stream.close()

    def write_to_wav(self, indata, frames, time, status):
        self.waveFile.writeframes(indata.tobytes())
  
def main():
    rclpy.init()
    cv2_recorder = CV2Record()
    rclpy.spin(cv2_recorder)
    cv2_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()