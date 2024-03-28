# SDP Team 12
# Author: Samuel Almeida
# Date created: 3/28/24
# Date last modified: 3/28/24
# A ROS node which runs the GUI for our robot

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32,Int32

import tkinter as tk
from tkinter import *
from tkinter import ttk
from tkinter.ttk import *
import subprocess
import json
import time
import threading
from threading import Thread

class RosGUI(Node):
    def __init__(self):
        super().__init__('gui')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        recording_timer_period = 1
        self.target_id = 0
        self.get_time_left = self.create_subscription(
            Float32,
            '/recording_time',
            self.update_recording_time,
        10)
        self.get_time_left = self.create_subscription(
            Int32,
            '/recording_max_time',
            self.update_max_recording_time,
        10)
        self.get_target = self.create_subscription(
            Int32,
            '/target_id',
            self.update_target_id,
        10)
        self.recording_time = 0
        self.max_recording_time = 10

    def update_recording_time(self,msg):
        self.recording_time = msg.data
        # print("Hi")
    def update_max_recording_time(self,msg):
        # print("hi")
        self.max_recording_time = msg.data

    def update_target_id(self,msg):
        # print("hi")
        self.target_id = msg.data

    # Add widgets here (e.g., labels, buttons, etc.)


    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1

def main(args=None):
    rclpy.init(args=args)

    gui = RosGUI()

    t1 = Thread(target=start_ros,args=[gui]).start()
    t2 = Thread(target=create_gui,args=[gui]).start()

    print("GUI startup completed successfully!")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

if __name__ == '__main__':
    main()

def start_ros(node : Node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
def create_gui(node : Node):
    gui = GUI(node)
    return gui

class GUI:
    def __init__(self, node : Node):
        self.node = node
        self.window = tk.Tk()
        self.window.geometry("800x600+10+20")

        btn1 = ttk.Button(text="Launch Target Tracking", command=lambda:self.launch(0))
        btn1.grid(row=0,column=0)

        btn2 = ttk.Button(text="Launch Nav", command=lambda:self.launch(1))
        btn2.grid(row=2,column=0)

        btn3 = ttk.Button(text="Launch Test", command=lambda:self.launch(2))
        btn3.grid(row=4,column=0)

        inputL = ttk.Entry(textvariable="enter list of targets",width=50)
        inputL.grid(row=6,column=1,columnspan=2)
        inputL.insert(0,string="Enter list of targets like this => [1,59,2]")

        btn3 = ttk.Button(text="Add Targets", command=lambda:self.launch(3,json.loads(inputL.get())))
        btn3.grid(row=6,column=0)

        btn4 = ttk.Button(text="Clear targets", command=lambda:self.launch(4))
        btn4.grid(row=8,column=0)

        btn5 = ttk.Button(text="Use Mic", command=lambda:self.launch(5))
        btn5.grid(row=10,column=0)

        inputR = ttk.Entry(textvariable="Recording timeout",width=50)
        inputR.grid(row=12,column=2,columnspan=3)
        inputR.insert(0,string="10")

        btn6 = ttk.Button(text="Update Recording Time", command=lambda:subprocess.Popen(['ros2','param','set','/target_pub','recording_timeout',str(inputR.get())]))
        btn6.grid(row=12,column=0)

        self.progress_label_var = StringVar()
        self.progress_label = ttk.Label(textvariable=self.progress_label_var)
        self.progress_label_var.set("Recording progress at 0.0%")
        self.progress_label.grid(row=14,column=0)
        self.progress_var = DoubleVar()
        self.progress = ttk.Progressbar(variable=self.progress_var,maximum=1)
        self.progress.grid(row=14,column=1,columnspan=5)

        # Add widgets here (e.g., labels, buttons, etc.)
        
        # Start the GUI event loop
        print("Initializing system...")
        # subprocess.run("./system_init.sh")
        open("targets.txt", "w").close() # clear text file

        self.update_progress()

        self.window.mainloop()

    def update_progress(self):
        # print("updating")
        # self.progress.value = 100
        # print(self.node.recording_time,self.node.max_recording_time)
        progress = round(float(self.node.recording_time)/float(self.node.max_recording_time),2)
        self.progress_var.set(float(self.node.recording_time)/float(self.node.max_recording_time))
        self.progress_label_var.set(f"Recording progress at: {progress*100}% for target: {self.node.target_id}",)
        self.window.after(1000, self.update_progress)

    def launch(self, arg: int, target_list: list = []):
        match arg:
            case 0:
                global tracking_pid
                tracking_cmd_list = ["ros2", "launch", "basic_mobile_robot", "target_tracking.launch.py"]
                tracking_pid = subprocess.Popen("./run_target_tracking.sh")
                # p2 = subprocess.run(["pgrep", "pub"])
            case 1:
                global nav_pid
                nav_pid = subprocess.Popen("./run_navigation.sh")
                print("Running nav")
                nav_pid = p.pid
            case 2:
                global test_pid
                test_pid= subprocess.Popen("./run_test.sh")
                print("Test")
                test_pid = p.pid
            case 3:
                # just make this case update the text file that you have or change the node with the input "target_list"
                fd = open("targets.txt", 'a')
                for i in target_list:
                    fd.write(str(i) + "\n")

                print("Updated list", target_list, type(target_list))
            case 4:
                print("Clearing all targets from log file...")
                open("targets.txt", "w").close() # clear text file
            case 5:
                p = subprocess.Popen("./mic.sh")
                print("Using microphone input")