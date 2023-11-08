import board
import pwmio
import digitalio
from time import sleep

dir1_pin = digitalio.DigitalInOut(board.GP27)
dir1_pin.direction = digitalio.Direction.OUTPUT

dir2_pin = digitalio.DigitalInOut(board.GP17)
dir2_pin.direction = digitalio.Direction.OUTPUT

pwm1_pin = pwmio.PWMOut(board.GP28, frequency = 50)
pwm2_pin = pwmio.PWMOut(board.GP16, frequency = 50)

def motor1_forward():
    dir1_pin_value = True

def motor1_reverse():
    dir1_pin.value = False

def motor1_speed(speed):
    pwm1_pin.duty_cycle = speed

def motor1_stop():
    pwm1_pin.duty_cycle = 0





def motor2_forward():
    dir2_pin_value = True

def motor2_reverse():
    dir2_pin.value = False

def motor2_speed(speed):
    pwm2_pin.duty_cycle = speed

def motor2_stop():
    pwm2_pin.duty_cycle = 0


def both_forward():
    dir1_pin_value = True
    sleep(1)
    dir2_pin_value = True

def both_reverse():
    dir1_pin.value = False
    dir2_pin.value = False

def both_speed(speed):
    pwm1_pin.duty_cycle = speed
    sleep(1)
    pwm2_pin.duty_cycle = speed

def both_stop():
    pwm1_pin.duty_cycle = 0
    sleep(1)
    pwm2_pin.duty_cycle = 0


while True:
    both_forward()
    both_speed(15000)
    sleep(2)

    both_stop()
    sleep(2)
    
    
    '''motor1_forward()
    motor1_speed(10000)
    sleep(2)

    motor1_stop()
    sleep(1)


    motor2_forward()
    motor2_speed(10000)
    sleep(2)

    motor2_stop()
    sleep(1)

    motor1_forward()
    motor1_speed(10000)
    motor2_forward()
    motor2_speed(10000)
    sleep(2)

    motor1_stop()
    motor2_stop()
    sleep(1)


    



    motor1_reverse()
    motor1_speed(10000)
    sleep(2)

    motor1_stop()
    sleep(1)


    motor2_reverse()
    motor2_speed(10000)
    sleep(2)

    motor2_stop()
    sleep(1)

    motor1_reverse()
    motor1_speed(10000)
    motor2_reverse()
    motor2_speed(10000)
    sleep(2)

    motor1_stop()
    motor2_stop()
    sleep(1)'''
    
    