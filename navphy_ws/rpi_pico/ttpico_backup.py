import usb_cdc
import time
import board
import pwmio
from adafruit_motor import servo
import json
import digitalio

################################################################
# select the serial Data port
################################################################

serial = usb_cdc.data
pwm1 = pwmio.PWMOut(board.GP0, duty_cycle=2 ** 8, frequency=330)
pwm2 = pwmio.PWMOut(board.GP1, duty_cycle=2 ** 8, frequency=330)
led = digitalio.DigitalInOut(board.GP21)
led.direction = digitalio.Direction.OUTPUT
my_servoy = servo.Servo(pwm2)
my_servox = servo.Servo(pwm1)
centerX = 10
centerY = 90
my_servox.angle = centerX
my_servoy.angle = centerY

tempx = centerX
tempy = centerY
data_inx,data_iny = centerX,centerY
k_x = 2
k_y = 1.5
integral_x = 0
integral_y = 0
d_x = 2
d_y = 2
vel = (0, 0)

# flag = 1
# prev_time = time.time()
offset = 0

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
                    vel = (float(translation[0])*(1920),float(translation[1])*(1080)) # convert pixel offset to angle
                    led_state = int(translation[2]) # get state of led to blink or not
                    print(translation)
                    if(led_state == 1):
                        led.value = True
                    elif led_state == 2:
                        if(time.monotonic_ns() - current_time > .5): #blink code
                            current_time = time.monotonic_ns()
                            led.value = not led.value

                
                # print(desired_angle)
                # error_x = (desired_angle[0]-my_servox.angle)
                # error_y = (desired_angle[1]-my_servoy.angle)
                # data_inx += translation[0] * (180/1920)
                # data_iny -= translation[1] * (180/1080)
                # data_inx = min(180,max(-180,data_inx))
                # data_iny = min(180,max(-180,data_iny))
                # print(my_servox.angle,my_servoy.angle, "data:",data_inx,data_iny)
                    data_inx -= vel[0]*dt*k_x + d_x*dt*10 # running two pd loops for x and y (feedback control)
                    data_iny += vel[1]*dt*k_y + d_y*dt*8

                    my_servox.angle = min(180,max(0,int(data_inx))) #these just prevent servo from going out of range
                    my_servoy.angle = min(120,max(0,int(data_iny)))  
                
                    myangle = [my_servox.angle-90,my_servoy.angle-90] #sends angle back to ros
                    print(dt,myangle)

                    serial.write(bytearray(json.dumps(myangle) + "\n", "utf-8")) #writes angles
        
                while(serial.out_waiting):
                    pass
            serial.flush() #flushes buffer to prevent serial issues

    d_x = my_servox.angle - tempx #gets derivative without dt 
    d_y = my_servoy.angle - tempy

    tempx = my_servox.angle # for derivative
    tempy = my_servoy.angle
