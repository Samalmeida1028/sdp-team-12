import usb_cdc
import time
import board
import pwmio
# from adafruit_motor import servo
import json
import motor_controller
from motor_controller import DFR0601, ChassisController
from adafruit_motor import servo

################################################################
# select the serial Data port
################################################################
TOP_SPEED = 65536
serial = usb_cdc.data


INA1 = board.GP21
INB1 = board.GP20
PWM1 = board.GP19

INA2 = board.GP26
INB2 = board.GP27
PWM2 = board.GP28

INA3 = board.GP7
INB3 = board.GP6
PWM3 = board.GP8

INA4 = board.GP3
INB4 = board.GP5
PWM4 = board.GP4

driver1 = DFR0601(PWM1,INA1,INB1,PWM2,INA2, INB2,80000)
driver2 = DFR0601(PWM3,INA3,INB3,PWM4,INA4, INB4,80000)

chassis_controls = ChassisController(driver1,driver2)



while True:
        if serial.in_waiting > 0:
            data_in = serial.readline()
        if(data_in):
            teleop = float(data_in.decode('utf-8'))
            if(teleop):
                print(float(teleop))

            driver1.change_speed(min(2000,max(-2000,-teleop*200)))
            driver2.change_speed(min(2000,max(-2000,-teleop*200)))
            print(driver1.speed_vector,driver2.speed_vector)

            
    