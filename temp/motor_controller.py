import board
import digitalio
import pwmio




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
        self.wheel1 = Wheel(PWM1_pin,INA1_pin,INB1_pin,freq)
        self.wheel2 = Wheel(PWM2_pin,INA2_pin,INB2_pin,freq)
        self.speed_vector = 0


    def change_speed(self, duty_cycle):
        self.speed_vector = duty_cycle
        self.wheel1.duty_cycle = duty_cycle
        self.wheel2.duty_cycle = duty_cycle
    
    # def set_speed(self, duty_cycle):
    #     self.PWM1.duty_cycle = int(abs(duty_cycle))
    #     self.PWM2.duty_cycle =int(abs(duty_cycle))
    #     self.speed_vector = int(duty_cycle)

    def set_forward(self):
        self.wheel1.set_forward()
        self.wheel2.set_forward()

    def set_backward(self):
        self.wheel1.set_backward()
        self.wheel2.set_backward()
    

    


class ChassisController:

    def __init__(self, motor_controller_1 : DFR0601, motor_controller_2 : DFR0601):
        self.motor_controller_1 = motor_controller_1
        self.motor_controller_2 = motor_controller_2



    # def forward(self,speed = ""):
    #     if(speed != ""):
    #         self.motor_controller_1.set_speed(int(speed))
    #         self.motor_controller_2.set_speed(int(speed))
    #     self.motor_controller_1.forward()
    #     self.motor_controller_2.forward()

    # def backward(self,speed):
    #     self.motor_controller_1.backward(int(speed))
    #     self.motor_controller_2.backward(int(speed))

    # def left(self,speed):
    #     self.motor_controller_1.left(int(speed))
    #     self.motor_controller_2.right(int(speed))


    # def right(self,speed):
    #     self.motor_controller_1.left(int(speed))
    #     self.motor_controller_2.right(int(speed))

