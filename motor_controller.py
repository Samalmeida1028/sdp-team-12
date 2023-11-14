import board
import digitalio
import pwmio


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


class ChassisController:

    def __init__(self, motor_controller_1 : DFR0601, motor_controller_2 : DFR0601):
        self.motor_controller_1 = motor_controller_1
        self.motor_controller_2 = motor_controller_2



    def forward(self,speed = ""):
        if(speed != ""):
            self.motor_controller_1.set_speed(int(speed*0xFFFF))
            self.motor_controller_2.set_speed(int(speed*0xFFFF))
        self.motor_controller_1.forward()
        self.motor_controller_2.forward()

    def backward(self,speed):
        self.motor_controller_1.backward(int(speed*0xFFFF))
        self.motor_controller_2.backward(int(speed*0xFFFF))

    def left(self,speed):
        self.motor_controller_1.left(int(speed*0xFFFF))
        self.motor_controller_2.left(int(speed*0xFFFF))


    def right(self,speed):
        self.motor_controller_1.right(int(speed*0xFFFF))
        self.motor_controller_2.right(int(speed*0xFFFF))



    