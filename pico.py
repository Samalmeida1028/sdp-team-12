"""
Read the Serial port to receive color data for the neopixel.


This uses the optional second serial port available in Circuitpython 7.x
Activate it in the boot.py file with the following code

import usb_cdc
usb_cdc.enable(console=True, data=True)
  # Initialize the rclpy library

Some boards might require disabling USB endpoints to enable the data port.
"""

import usb_cdc
import time
import board
import pwmio
from adafruit_motor import servo
import json

################################################################
# select the serial Data port
################################################################

serial = usb_cdc.data
pwm1 = pwmio.PWMOut(board.GP0, duty_cycle=2 ** 15, frequency=50)
pwm2 = pwmio.PWMOut(board.GP1, duty_cycle=2 ** 12, frequency=50)
my_servoy = servo.Servo(pwm2)
my_servox = servo.Servo(pwm1)
center = 90
my_servox.angle = center
my_servoy.angle = center

################################################################
# init board's LEDs for visual output
# replace with your own pins and stuff
################################################################

# pix = None
# if hasattr(board, "NEOPIXEL"):
#     import neopixel
#     pix = neopixel.NeoPixel(board.NEOPIXEL, 1)
#     pix.fill((32, 16, 0))
# else:
#     print("This board is not equipped with a Neopixel.")

tempx = center
tempy = center
data_inx,data_iny = center,center
integral_x = 0
integral_y = 0
while True:
    # read the secondary serial line by line when there's data
    # note that this assumes that the host always sends a full line
    if serial.in_waiting > 0:
        data_in = serial.readline()
        if(data_in):
            translation = json.loads(data_in.decode('utf-8'))
            if my_servox.angle is not 180 and my_servox.angle is not 0:
                integral_x += (my_servox.angle-tempx) * .01
            if my_servoy.angle is not 180 and my_servoy is not 0:
                integral_y += (my_servoy.angle - tempy) * .01
            derivative_x = ((my_servox.angle-tempx)*3) 
            derivative_y = ((my_servoy.angle-tempy)*3)
            data_inx += ((translation[0] * (180/640))*.8) + derivative_x 
            data_iny -= ((translation[1] * (180/480))*.08) + derivative_y
            print(integral_x,integral_y,derivative_x,derivative_y)


            my_servox.angle = min(180,max(0,data_inx))
            my_servoy.angle = min(120,max(0,data_iny))

            tempx = my_servox.angle
            tempy = my_servoy.angle
        else:
            my_servox.angle = min(180,max(0,tempx))
            my_servoy.angle = min(120,max(0,tempy))

    int(data_inx),int(data_iny)
        # if(data_in):  # Initialize the rclpy library

        #     data_inx,data_iny = data_in.decode('utf-8').split(" ")
        #     data_inx = float(data_inx.strip())*180
        #     data_iny = float(data_iny.strip())*180
        #     data_inx = max(min(int(data_inx), 180),0)
        #     data_iny = max(min(int(data_iny),180),0)
        #     print(data_inx,data_iny)
        #     my_servo.angle = data_iny
        #     my_servo2.angle = data_inx
        #     tempy= int(data_iny)
        #     tempx= int(data_inx)
        # else:
        #     my_servo.angle = tempy
        #     my_servo2.angle = tempx
        

        # try to convert the data to a dict (with JSON)
        # data = None
        # if data_iny and data_inx and int(data_iny) < 100 and int(data_inx) < 100:
        #     my_servo.angle = int(data_iny)
        #     my_servo2.angle = int(data_inx)
        #     tempy= int(data_iny)
        #     tempx= int(data_inx)
        #     print(data_inx,data_iny)
        # else:
        #     my_servo.angle = tempy
        #     my_servo2.angle = tempx


    # for angle in range(1, 100, 5):  # 0 - 180 degrees, 5 degrees at a time
    #     my_servo.angle = 0
    #     time.sleep(0.01)
    # for angle in range(1, 100, 5):  # 0 - 180 degrees, 5 degrees at a time
    #     my_servo2.angle = 0
    #     time.sleep(0.01)
    # for angle in range(100, 0, -5):  # 0 - 180 degrees, 5 degrees at a time
    #     my_servo.angle = 0
    #     time.sleep(0.01)
    # for angle in range(100, 0, -5):  # 0 - 180 degrees, 5 degrees at a time
    #     my_servo2.angle = 0
    #     time.sleep(0.01)
            
                

    # this is where the rest of your code goes
    # if the code does a lot you don't need a call to sleep, but if possible
    # it's good to have the microcontroller sleep from time to time so it's
    # not constantly chugging
    time.sleep(0.01)