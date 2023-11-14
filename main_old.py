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

p_gain = .12
d_gain = 7.8
i_gain = 1.9

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
            # print(translation[0])
            if(translation[0]**2 + translation[1]**2 > 10):
                if my_servox.angle < 180 and my_servox.angle > 0:
                    integral_x += (my_servox.angle-tempx)
                if my_servoy.angle < 180 and my_servoy.angle > 0:
                    integral_y += (my_servoy.angle - tempy)
                derivative_x = ((my_servox.angle-tempx)) 
                derivative_y = ((my_servoy.angle-tempy))
                data_inx += (translation[0] * (180/1920))*p_gain + derivative_x*d_gain + integral_x*i_gain
                data_iny -= (translation[1] * (180/1080))*p_gain*.5 - derivative_y*d_gain - integral_y*i_gain



                my_servox.angle = min(180,max(0,data_inx))
                my_servoy.angle = min(120,max(0,data_iny))

                tempx = my_servox.angle
                tempy = my_servoy.angle
        else:
            my_servox.angle = min(180,max(0,tempx))
            my_servoy.angle = min(120,max(0,tempy))

    int(data_inx),int(data_iny)
    time.sleep(0.01)