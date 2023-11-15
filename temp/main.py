import usb_cdc
import time
import board
import pwmio
# from adafruit_motor import servo
import json
import motor_controller
from motor_controller import ChassisController, Wheel
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


# driver1 = DFR0601(PWM1,INA1,INB1,PWM3,INA3, INB3,80000) # wiring is different, so calling opposite pins 11/14/23
# driver2 = DFR0601(PWM2,INA2,INB2,PWM4,INA4, INB4,80000)

wheel1 = Wheel(PWM1,INA1,INB1,freq=80000)
wheel2 = Wheel(PWM2,INA2,INB2,freq=80000)
wheel3 = Wheel(PWM3,INA3,INB3,freq=80000)
wheel4 = Wheel(PWM4,INA4,INB4,freq=80000)


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
                wheel1.set_forward()
                #  wheel2.set_backward()
                #  wheel3.set_forward()
                #  wheel4.set_backward()
            # elif dis_phase == 1:
            #     wheel1.set_forward()
            #     wheel2.set_backward()
            #     wheel3.set_forward()
            #     wheel4.set_backward()
            # elif dis_phase == 2:
            #     wheel1.set_forward()
            #     wheel2.set_backward()
            #     wheel3.set_forward()
            #     wheel4.set_backward()
            # elif dis_phase == 3:
            #     wheel1.set_forward()
            #     wheel2.set_backward()
            #     wheel3.set_forward()
            #     wheel4.set_backward()
            # elif dis_phase == 4:
            #     wheel1.set_forward()
            #     wheel2.set_backward()
            #     wheel3.set_forward()
            #     wheel4.set_backward()
            # elif dis_phase == 5:
            #     wheel1.set_forward()
            #     wheel2.set_backward()
            #     wheel3.set_forward()
            #     wheel4.set_backward()
            # elif dis_phase == 6:
            #     wheel1.set_forward()
            #     wheel2.set_backward()
            #     wheel3.set_forward()
            #     wheel4.set_backward()
            # elif dis_phase == 7:
            #     wheel1.set_forward()
            #     wheel2.set_backward()
            #     wheel3.set_forward()
            #     wheel4.set_backward()
            else:
                pass
            
            # print("\\\\")
            wheel1.duty_cycle = (int(mag*30000))
            wheel2.duty_cycle = (int(mag*30000))
            wheel3.duty_cycle = (int(mag*30000))
            wheel4.duty_cycle = (int(mag*30000))
            print(wheel1.INB.value)

            # print(dis_phase, driver1.wheel1.INA.value)
            # print("////")




            
    