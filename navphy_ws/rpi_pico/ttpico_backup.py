import usb_cdc
import time
import board
import pwmio
from adafruit_motor import servo
import json
import digitalio
import math

import atexit


################################################################
# select the serial Data port
################################################################
centerY = 90 # degrees
centerX = 90 # degrees

serial = usb_cdc.data

pwm2 = pwmio.PWMOut(board.GP0, duty_cycle=2 ** 15, frequency=330)
my_servoy = servo.Servo(pwm2)
my_servoy.angle = centerY

pwm1 = pwmio.PWMOut(board.GP1, duty_cycle=2 ** 15, frequency=330)
my_servox = servo.Servo(pwm1)
my_servox.angle = centerX

led = digitalio.DigitalInOut(board.GP9)
led.direction = digitalio.Direction.OUTPUT

tempx = centerX
tempy = centerY
data_inx,data_iny = centerX,centerY
k_x = .1
k_y = .1
integral_x = 0
integral_y = 0
d_x = 2
d_y = 2
vel = (0, 0)

offset_x_accel = 0
offset_y_accel = 0
# flag = 1
prev_time = time.time()
dt_prev_time = time.monotonic_ns()
offset = [0,0]


current_time = time.monotonic_ns()
target_seen: bool = False
finishing_recording: bool = False
target_last_seen = 0.0

def turn_servo_off():
    global pwm1
    pwm1 = None
    global pwm2
    pwm2 = None

atexit.register(turn_servo_off)

print("STARTIN")
recording_led_state = 0
while True:
    # print("hi")

    ##################### DEBUG ######################
    # print("Here")
    t = time.time() - prev_time
    ##################################################

    
    if serial.in_waiting > 0:
        data_in = serial.readline()
        if data_in:
            dt_prev_time = time.time()
            if json.loads(data_in.decode()) == "Type":
                # print("Asking type")
                serial.write(bytearray(json.dumps("tracking") + "\n"))
            else:

                translation = json.loads(data_in.decode('utf-8'))
                if translation == "center":
                    my_servox.angle = centerX
                    my_servoy.angle = centerY
                    tempx = my_servox.angle
                    tempy = my_servoy.angle
                    led.value = 0

                else:
                    dt = time.monotonic_ns()-dt_prev_time
                    # print(dt*1e-14)
                    if(dt > .5):
                        dt = .02
                    vel = (180*float(translation[0])/(1920),180*float(translation[1])/(1080)) # convert pixel offset to angle
                    led_state = int(translation[2]) # get state of led to blink or not
                    # print(translation)
                    recording_led_state = led_state
                    if(led_state == 1 or led_state == 2):
                        target_last_seen = time.monotonic_ns()*1e-9
                    if math.fabs(vel[0]) + math.fabs(vel[1]) > 5:
                        offset_x_accel += (1.4*vel[0] + 30000*(d_x))*dt
                        offset_y_accel += (1.1*vel[1] + 2000*(d_y))*dt
                        myangle = [my_servox.angle-90,my_servoy.angle-90] #sends angle back to ros
                    else:
                        myangle = [tempx-90,tempy-90] #sends angle back to ros

                    serial.write(bytearray(json.dumps(myangle) + "\n", "utf-8")) #writes angles
        
                while(serial.out_waiting):
                    pass
            serial.flush() #flushes buffer to prevent serial issues

    d_x = my_servox.angle - tempx #gets derivative without dt 
    d_y = my_servoy.angle - tempy
    my_servox.angle = min(160,max(0,my_servox.angle - offset_x_accel)) #these just prevent servo from going out of range
    my_servoy.angle = min(120,max(0,my_servoy.angle + offset_y_accel))
    offset_x_accel *= .5
    offset_y_accel *= .3

    tempx = my_servox.angle # for derivative
    tempy = my_servoy.angle

    if recording_led_state == 0:
        led.value = 0
    if recording_led_state == 1:
        led.value = 1
    if recording_led_state == 2:
        if(time.monotonic_ns()*1e-9 % .5 == 0):
            led.value = 1 - led.value

    # print(target_seen,(time.monotonic_ns()*1e-9)-target_last_seen,t)