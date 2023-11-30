import board as b
import busio
import adafruit_lsm6ds.lsm6dsox
import time
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX


i2c=(busio.I2C(sda=b.GP2, scl=b.GP3))  # uses board.SCL and board.SDA
sox = LSM6DSOX(i2c)

# LSM6DSOX.reset(sox)
vel = [0,0,0]
pos = [0,0,0]
sox.accelerometer_range = 2
sox.accelerometer_data_rate = 10
sox.high_pass_filter = 1
print(sox.accelerometer_data_rate)
while True:
    for i in range(3):
        if i != 2:
            vel[i] += sox.acceleration[i]
        pos[i] += vel[i]
    print(sox.acceleration,vel, pos)
