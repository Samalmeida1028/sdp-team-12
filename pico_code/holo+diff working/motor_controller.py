import board
import digitalio
import pwmio


#NEED TO GET NEW CODE OFF PICO

class Wheel:
    def __init__(self, PWM, INA, INB,freq) -> None:
        self.PWM = pwmio.PWMOut(PWM,duty_cycle=0,frequency=freq)
        self.INA = digitalio.DigitalInOut(INA)
        self.INB = digitalio.DigitalInOut(INB)
        self.INA.switch_to_output()
        self.INB.switch_to_output()
        self.INA.value = False
        self.INB.value = False
        self.duty_cycle = 0

    def set_forward(self):
        self.INA.value = False
        self.INB.value = True
    def set_backward(self):
        self.INA.value = True
        self.INB.value = False
    def stop(self):
        self.INA.value = False
        self.INB.value = False


class DFR0601:


    def __init__(self, PWM1_pin, INA1_pin, INB1_pin, PWM2_pin, INA2_pin, INB2_pin, freq):
        self.PWM1 = pwmio.PWMOut(PWM1_pin,duty_cycle=0,frequency=freq)
        self.INA1 = digitalio.DigitalInOut(INA1_pin)
        self.INB1 = digitalio.DigitalInOut(INB1_pin)
        self.PWM2 = pwmio.PWMOut(PWM2_pin,duty_cycle=0,frequency=freq)
        self.INA2 = digitalio.DigitalInOut(INA2_pin)
        self.INB2 = digitalio.DigitalInOut(INB2_pin)
        self.INA1.switch_to_output()
        self.INB1.switch_to_output()
        self.INA2.switch_to_output()
        self.INB2.switch_to_output()
        self.INA1.value = False
        self.INA2.value = False
        self.INB1.value = False
        self.INB2.value = False
        self.frequency = freq
        self.speed_vector = 0


    def change_speed(self, duty_cycle):
        self.speed_vector += duty_cycle
        if(abs(self.speed_vector) > 65000):
            if(self.speed_vector < 0):
                self.speed_vector = -65000
            else:
                self.speed_vector = 65000
        self.PWM1.duty_cycle = max(11000,abs(int(self.speed_vector)))
        self.PWM2.duty_cycle = max(11000,abs(int(self.speed_vector)))
        self.PWM1.duty_cycle = round(self.PWM1.duty_cycle)
        self.PWM1.duty_cycle = round(self.PWM2.duty_cycle)
        if(self.speed_vector < 0):
            self.forward()
        else:
            self.backward()

    
    def set_speed(self, duty_cycle):
        self.PWM1.duty_cycle = int(abs(duty_cycle))
        self.PWM2.duty_cycle =int(abs(duty_cycle))
        self.speed_vector = int(duty_cycle)
        



    def forward(self):
        self.INB1.value = True
        self.INB2.value = True
        self.INA1.value = False
        self.INA2.value = False

    def backward(self):
        self.INB1.value = False
        self.INB2.value = False
        self.INA1.value = True
        self.INA2.value = True

    def left(self, speed):
        self.INB1.value = False
        self.INB2.value = True
        self.INA1.value = True
        self.INA2.value = False
        self.PWM1.duty_cycle= self.PWM2.duty_cycle = speed

    def right(self, speed):
        self.INB1.value = True
        self.INB2.value = False
        self.INA1.value = False
        self.INA2.value = True
        self.PWM1.duty_cycle= self.PWM2.duty_cycle = speed
    
    def stop(self):
        self.INA1.value = self.INA2.value = self.INB1.value = self.INB2.value = False
        self.PWM1.duty_cycle = self.PWM2.duty_cycle = 0
    
    def wheel_1_forward(self):
        self.INA1.value = False
        self.INB1.value = True
    def wheel_1_backward(self):
        self.INA1.value = True
        self.INB1.value = False
    def wheel_2_forward(self):
        self.INA2.value = False
        self.INB2.value = True
    def wheel_2_backward(self):
        self.INA2.value = True
        self.INB2.value = False
    def wheel_1_stop(self):
        self.INA1.value = False
        self.INB1.value = False
    def wheel_2_stop(self):
        self.INA2.value = False
        self.INB2.value = False

    


class ChassisController:

    def __init__(self, motor_controller_1 : DFR0601, motor_controller_2 : DFR0601):
        self.motor_controller_1 = motor_controller_1
        self.motor_controller_2 = motor_controller_2


    def set_fr_wheel(self, dir : str = 'forward', speed : int = 15000):
        if ( dir == 'forward'):
            self.motor_controller_1.wheel_1_forward()
        if ( dir == 'backward'):
            self.motor_controller_1.wheel_1_backward()
        if ( dir == 'stop'):
            self.motor_controller_1.wheel_1_stop()
        self.motor_controller_1.PWM1.duty_cycle = speed

    def set_br_wheel(self, dir : str = 'forward', speed : int = 15000):
        if ( dir == 'forward'):
            self.motor_controller_1.wheel_2_forward()
        if ( dir == 'backward'):
            self.motor_controller_1.wheel_2_backward()
        if ( dir == 'stop'):
            self.motor_controller_1.wheel_2_stop()
        self.motor_controller_1.PWM2.duty_cycle = speed

    def set_fl_wheel(self, dir : str = 'forward', speed : int = 15000):
        if ( dir == 'forward'):
            self.motor_controller_2.wheel_1_forward()
        if ( dir == 'backward'):
            self.motor_controller_2.wheel_1_backward()
        if ( dir == 'stop'):
            self.motor_controller_2.wheel_1_stop()
        self.motor_controller_2.PWM1.duty_cycle = speed

    def set_bl_wheel(self, dir : str = 'forward', speed : int = 15000):
        if ( dir == 'forward'):
            self.motor_controller_2.wheel_2_forward()
        if ( dir == 'backward'):
            self.motor_controller_2.wheel_2_backward()
        if ( dir == 'stop'):
            self.motor_controller_2.wheel_2_stop()
        self.motor_controller_2.PWM2.duty_cycle = speed



