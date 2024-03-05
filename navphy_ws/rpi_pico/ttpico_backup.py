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
center = 80
my_servox.angle = center
my_servoy.angle = center

tempx = center
tempy = center
data_inx,data_iny = center,center
integral_x = 0
integral_y = 0

# flag = 1
count = 0
offset = 0

start_time = time.monotonic_ns()
while True:
    # print("hi")
    if serial.in_waiting > 0:
        count = 0
        data_in = serial.readline()
        if data_in:
            if json.loads(data_in.decode()) == "Type":
                # print("Asking type")
                serial.write(bytearray(json.dumps("tracking") + "\n"))
            else:
                translation = json.loads(data_in.decode('utf-8'))

                if translation == "center":
                    my_servox.angle = (float(center)*(1080/1920),float(center)*(1080/1920))
                    my_servoy.angle = (float(center)*(1080/1920),float(center)*(1080/1920))
                else:
                    desired_angle = (float(translation[0])*(1080/1920),float(translation[1])*(1080/1920))
                
                print(desired_angle)
                # error_x = (desired_angle[0]-my_servox.angle)
                # error_y = (desired_angle[1]-my_servoy.angle)
                # data_inx += translation[0] * (180/1920)
                # data_iny -= translation[1] * (180/1080)
                # data_inx = min(180,max(-180,data_inx))
                # data_iny = min(180,max(-180,data_iny))
                # print(my_servox.angle,my_servoy.angle, "data:",data_inx,data_iny)
                data_inx += desired_angle[0]*.01
                data_iny -= desired_angle[1]*.01

                my_servox.angle = min(180,max(0,int(data_inx)))
                my_servoy.angle = min(120,max(0,int(data_iny)))  
            
                myangle = [90-my_servox.angle,my_servoy.angle-90]

                serial.write(bytearray(json.dumps(myangle) + "\n", "utf-8"))
        
                while(serial.out_waiting):
                    pass
            serial.flush()
                
    tempx = my_servox.angle
    tempy = my_servoy.angle