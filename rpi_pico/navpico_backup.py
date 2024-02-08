import usb_cdc
import time
import board
import pwmio

# from adafruit_motor import servo
import json
import motor_controller
from motor_controller import DFR0601, ChassisController, Encoder
import math
import adafruit_lsm6ds.lsm6dsox
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import busio

################################################################
# select the serial Data port
################################################################
TOP_SPEED = 65536
WHEEL_SEP = 0.3683
WHEEL_RADIUS = 0.0508

serial = usb_cdc.data
serial.write_timeout = 1

i2c = busio.I2C(sda=board.GP14, scl=board.GP15)
sox = LSM6DSOX(i2c)

INA1 = board.GP19
INB1 = board.GP20
PWM1 = board.GP21

INA2 = board.GP28
INB2 = board.GP27
PWM2 = board.GP26

INA3 = board.GP8
INB3 = board.GP7
PWM3 = board.GP6

INA4 = board.GP5
INB4 = board.GP4
PWM4 = board.GP3

D1 = board.GP0
D2 = board.GP1
D3 = board.GP10
D4 = board.GP11
D5 = board.GP17
D6 = board.GP16
# D7 = board.GP18
# D8 = board.GP22

driver1 = DFR0601(
    PWM1, INB1, INA1, PWM3, INB3, INA3, 5000
)  # wiring is different, so calling opposite pins 11/14/23
driver2 = DFR0601(PWM2, INB2, INA2, PWM4, INB4, INA4, 5000)

chassis = ChassisController(driver1, driver2)

enc1 = Encoder(D1, D2, 1)
enc2 = Encoder(D3, D4, 1)
enc3 = Encoder(D5, D6, 1)
# enc4 = rotaryio.IncrementalEncoder(D7, D8)
encoder_data = {}

encoders = [enc1, enc2, enc3]

last_position = None

data_in = []
while True:
    # print(chassis.motor_controller_1.INA1.value, chassis.motor_controller_1.INB1.value)
    if serial.in_waiting > 0:
        data_in = serial.readline()
        # print(data_in.decode())
    if data_in:
        teleop = json.loads(data_in.decode("utf-8"))
        print(teleop)

        # mag = math.sqrt(teleop[1]**2 + teleop[0]**2)
        # phase = math.atan2(teleop[0],teleop[1])
        # print(teleop)
        if teleop:
            #print(teleop)
            K = 30000
            lin_x = teleop[0]
            ang_z = teleop[1]

            v1 = int(-K * (lin_x + (ang_z * (WHEEL_SEP / 2))))
            v2 = int(-K * (lin_x - (ang_z * (WHEEL_SEP / 2))))
            v3 = v1
            v4 = v2

            motors = [v1, v2, v3, v4]
            print(teleop, motors)

            chassis.set_fl_wheel(v1)
            chassis.set_fr_wheel(v2)
            chassis.set_bl_wheel(v3)
            chassis.set_br_wheel(v4)
        else:
            v1 = 0
            v2 = 0
            v3 = 0
            v4 = 0
            motors = [v1, v2, v3, v4]
            chassis.set_fl_wheel(v1)
            chassis.set_fr_wheel(v2)
            chassis.set_bl_wheel(v3)
            chassis.set_br_wheel(v4)

    encoder_data.update({"BL": enc1.get_data(time.monotonic_ns())})
    encoder_data.update({"FL": enc2.get_data(time.monotonic_ns())})
    encoder_data.update({"FR": enc3.get_data(time.monotonic_ns())})

    imu_data = sox.gyro
    imu_data2 = sox.acceleration
    sensor_data = {
        "Encoder": encoder_data,
        "IMU": {
            "Gyro": {"r": imu_data[0], "p": imu_data[1], "y": -imu_data[2]},
            "Accel": {"x": imu_data2[0], "y": imu_data2[1], "z": -imu_data2[2]},
        },
    }
    remaining = bytes(json.dumps(sensor_data) + "\n", "utf-8")
    # print(sensor_data["Encoder"])
    # print(sensor_data["Encoder"]["BL"]["Vel"], 
    #       sensor_data["Encoder"]["FL"]["Vel"], 
    #       sensor_data["Encoder"]["FR"]["Vel"])

    n = serial.write(remaining)
    #print(n)
    while serial.out_waiting:
        pass
