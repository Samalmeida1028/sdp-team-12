import serial
import pynput
from pynput import keyboard
from pynput.keyboard import Key

sport = serial.Serial(port='/dev/ttyACM1', baudrate=9600,parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS)
# sport.open()
sendingdatax = 0
sendingdatay = 0


def on_key_release(key):
    global sendingdatax
    global sendingdatay
    if key == Key.right:
        sendingdatax +=2
    elif key == Key.left:
        sendingdatax -= 2
    elif key == Key.up:
        sendingdatay += 2
    elif key == Key.down:
        sendingdatay -= 2
    elif key == Key.esc:
        exit()

    print(sendingdatax/180,sendingdatay/180)
    sport.write(bytearray(str(sendingdatay/180)+" "+ str(sendingdatax/180)+'\n',encoding='UTF-8'))


with keyboard.Listener(on_press=on_key_release) as listener:
    listener.join()

