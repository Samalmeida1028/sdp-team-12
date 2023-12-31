import serial
import json
import sys

import pygame
joysticks = []

pygame.joystick.init()
pygame.init()


# Variables for locations in list #
AXIS_PITCH = 1
AXIS_ROLL = 0
AXIS_THROTTLE = 2
BUTTON_TRIGGER = BUTTON_1 = 3
BUTTON_2 = 4
BUTTON_3 = 5
BUTTON_4 = 6
BUTTON_5 = 7

sport = serial.Serial(port='COM12', baudrate=9600, write_timeout=1,parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS)


# Must be called first, initializes joysticks #
def init():
    for joy in range(pygame.joystick.get_count()):
        joysticks.append(pygame.joystick.Joystick(joy))
        joysticks[joy].init()




def main():
    while(1):
        pygame.event.get()
        for stick in joysticks:
                    sport.write(bytearray(json.dumps([stick.get_axis(0), stick.get_axis(1)]) + "\n", encoding="utf-8"))
        ser = sport.readline()
        print(ser.decode('utf-8'))

                    



if __name__ == "__main__":
      init()
      main()