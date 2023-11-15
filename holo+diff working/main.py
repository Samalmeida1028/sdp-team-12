import usb_cdc
import time
import board
import pwmio
# from adafruit_motor import servo
import json
import motor_controller
from motor_controller import DFR0601,ChassisController
from adafruit_motor import servo
import math

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


driver1 = DFR0601(PWM1,INA1,INB1,PWM3,INA3, INB3,80000) # wiring is different, so calling opposite pins 11/14/23
driver2 = DFR0601(PWM2,INA2,INB2,PWM4,INA4, INB4,80000)

chassis = ChassisController(driver1,driver2)



data_in = []


def steps(values : int, v : float) ->  int:

    ret = min(math.floor((math.floor(((v + math.pi)*values)/2*math.pi)/10)+.5),7)
    return ret

 

while True:
        if serial.in_waiting > 0:
            data_in = serial.readline()
        if(data_in):
            teleop = json.loads(data_in.decode('utf-8'))

            mag = math.sqrt(teleop[1]**2 + teleop[0]**2)
            phase = math.atan2(teleop[1],teleop[0])
            dis_phase = steps(8, phase)

            if dis_phase == 0:
                 driver1.forward()
                 driver2.backward()
            elif dis_phase == 1:
                 driver1.forward()
                 driver2.stop()
                 driver2.INA1.value = True
            elif dis_phase == 2:
                 driver1.forward()
                 driver2.forward()
            elif dis_phase == 3:
                 driver2.forward()
                 driver1.stop()
            elif dis_phase == 4:
                 driver1.backward()
                 driver2.forward()
            elif dis_phase == 5:
                 driver2.backward()
                 driver1.stop()
            elif dis_phase == 6:
                 driver1.backward()
                 driver2.backward()
            elif dis_phase == 7:
                 driver1.backward()
                 driver2.stop()
            else:
                pass
            driver1.set_speed(mag*20000)
            driver2.set_speed(mag*20000)
            # print("\\\\")
            print(dis_phase)
            # print("////")




            
    