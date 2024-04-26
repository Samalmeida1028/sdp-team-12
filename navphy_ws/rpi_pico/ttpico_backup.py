import usb_cdc
import time
import board
import pwmio
from adafruit_motor import servo
import json
import digitalio
import math


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

led = digitalio.DigitalInOut(board.GP21)
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

# flag = 1
# prev_time = time.time()
offset = [0,0]

dt = time.monotonic_ns()*1e-14
current_time = time.monotonic_ns()
while True:
    dt = (time.monotonic_ns()*1e-14)-dt
    # print("hi")

    ###################### DEBUG ######################
    # t = time.time() - prev_time
    # if t > 0.5:
    #     led.value ^= 1
    #     prev_time = time.time()
    ###################################################
    
    if serial.in_waiting > 0:
        data_in = serial.readline()
        prev_time = time.time()
        if data_in:
            if json.loads(data_in.decode()) == "Type":
                # print("Asking type")
                serial.write(bytearray(json.dumps("tracking") + "\n"))
            else:

                translation = json.loads(data_in.decode('utf-8'))
                if translation == "center":
                    my_servox.angle = centerX
                    my_servoy.angle = centerY - 5
                    led.value = 0

                else:
                    vel = (180*float(translation[0])/(1920),180*float(translation[1])/(1080)) # convert pixel offset to angle
                    led_state = int(translation[2]) # get state of led to blink or not
                    print(translation)
                    if(led_state == 1):
                        led.value = True
                    elif led_state == 2:
                        if(time.monotonic_ns() - current_time > .5): #blink code
                            current_time = time.monotonic_ns()
                            led.value = not led.value

                    mult_const_x = 1 if vel[0] > 0 else -1
                    vel_sqrt_x = mult_const_x*math.sqrt(math.fabs(vel[0]))
                    mult_const_y = 1 if vel[1] > 0 else -1
                    vel_sqrt_y = mult_const_y*math.sqrt(math.fabs(vel[1]))
                    if math.fabs(vel[0]) + math.fabs(vel[1]) > 5:
                        my_servox.angle = min(150,max(0,my_servox.angle - .3*vel_sqrt_x)) #these just prevent servo from going out of range
                        my_servoy.angle = min(100,max(0,my_servoy.angle + .15*vel_sqrt_y) )  
                        myangle = [my_servox.angle-90,my_servoy.angle-90] #sends angle back to ros
                        print(dt,myangle)
                    else:
                        myangle = [tempx-90,tempy-90] #sends angle back to ros

                    serial.write(bytearray(json.dumps(myangle) + "\n", "utf-8")) #writes angles
        
                while(serial.out_waiting):
                    pass
            serial.flush() #flushes buffer to prevent serial issues

    d_x = my_servox.angle - tempx #gets derivative without dt 
    d_y = my_servoy.angle - tempy

    tempx = my_servox.angle # for derivative
    tempy = my_servoy.angle
