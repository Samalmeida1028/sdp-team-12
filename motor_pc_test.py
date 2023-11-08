
import serial
import json

ser = serial.Serial(
             'COM4',
             baudrate=115200,
             timeout=0.01)
      
    # Used to convert between ROS and OpenCV images


while(True):
  data = "Hello Pico"
   
  ser.write(bytearray(data +"\n",encoding="utf-8"))
  