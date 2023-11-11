
import serial
import json
import pynput
from pynput.keyboard import Key, Listener

ser = serial.Serial(
             'COM4',
             baudrate=115200,
             timeout=0.01)
      
    # Used to convert between ROS and OpenCV images


movement_vector = [0.0,0.0]

# while(True):
#   data = "Hello Pico"
   
#   ser.write(bytearray(data +"\n",encoding="utf-8"))
def on_press(key):
    #print('{0} pressed'.format(
        #key))
    check_key(key)

def on_release(key):
    #print('{0} release'.format(
       # key))
    if key == Key.esc:
        # Stop listener
        return False

def check_key(key):
    if key is Key.up:
      movement_vector[0]+=.01
    if key is Key.down:
      movement_vector[0]-=.01
    movement_vector[0]=min(max(-.9,movement_vector[0]),.9)
    ser.write(bytearray(json.dumps(movement_vector) +"\n",encoding="utf-8"))
         
      

# Collect events until released
with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()
  