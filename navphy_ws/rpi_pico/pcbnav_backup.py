import usb_cdc
import time
import board

import json
from motor_controller import DFR0601, ChassisController, Encoder
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import busio

################################################################
# select the serial Data port
################################################################
TOP_SPEED = 65536
WHEEL_SEP = 0.56
WHEEL_RADIUS = 0.0476

serial = usb_cdc.data
serial.write_timeout = 1

i2c = busio.I2C(sda=board.GP14, scl=board.GP15)
sox = LSM6DSOX(i2c)

INA1 = board.GP6
INB1 = board.GP5
PWM1 = board.GP4

INA2 = board.GP9
INB2 = board.GP8
PWM2 = board.GP7

INA3 = board.GP18
INB3 = board.GP17
PWM3 = board.GP16

INA4 = board.GP21
INB4 = board.GP20
PWM4 = board.GP19

D1 = board.GP2
D2 = board.GP3
D3 = board.GP0
D4 = board.GP1
D5 = board.GP10
D6 = board.GP11
D7 = board.GP12
D8 = board.GP13

driver1 = DFR0601(
    PWM1, INB1, INA1, PWM2, INB2, INA2, 5000
)  # wiring is different, so calling opposite pins 11/14/23
driver2 = DFR0601(PWM3, INB3, INA3, PWM4, INB4, INA4, 5000)

chassis = ChassisController(driver1, driver2)

enc_fr = Encoder(D2, D1, 1) # FR
enc_br = Encoder(D4, D3, 1) # BR
enc_bl = Encoder(D5, D6, 1) # BL
enc_fl = Encoder(D7, D8, 1) # FL

encoder_data = {}

last_position = None

data_in = []

def debug():
    K_p = 35000
    K_z = 5
    K_l = 1.25

    lin_x = 0.5
    ang_z = 0.0

    v1 = int(-K_p * ((lin_x*K_l) - ((ang_z*K_z) * (WHEEL_SEP / 2))))
    v2 = int(-K_p * ((lin_x*K_l) + ((ang_z*K_z) * (WHEEL_SEP / 2))))
    v3 = v1
    v4 = v2

    motors = [v1, v2, v3, v4]
    #print(motors)

    chassis.set_fl_wheel(v1)
    chassis.set_fr_wheel(v2)
    chassis.set_bl_wheel(v3)
    chassis.set_br_wheel(v4)

    # time.sleep(0.5)

    # chassis.set_fl_wheel(0)
    # chassis.set_fr_wheel(0)
    # chassis.set_bl_wheel(0)
    # chassis.set_br_wheel(0)

    # time.sleep(0.5)

    encoder_data.update({"FR": enc_fr.get_data(time.monotonic_ns())}) # FR
    encoder_data.update({"BR": enc_br.get_data(time.monotonic_ns())}) # BR
    encoder_data.update({"BL": enc_bl.get_data(time.monotonic_ns())}) # BL
    encoder_data.update({"FL": enc_fl.get_data(time.monotonic_ns())}) # FL

    imu_data = sox.gyro
    imu_data2 = sox.acceleration
    sensor_data = {
        "Encoder": encoder_data,
        "IMU": {
            "Gyro": {"r": imu_data[0], "p": imu_data[1], "y": -imu_data[2]},
            "Accel": {"x": imu_data2[0], "y": imu_data2[1], "z": -imu_data2[2]},
        },
    }

    #print(sensor_data["IMU"])

    print(sensor_data["Encoder"]["BL"]["Vel"], 
        sensor_data["Encoder"]["FL"]["Vel"], 
        sensor_data["Encoder"]["FR"]["Vel"],
        sensor_data["Encoder"]["BR"]["Vel"])
    
# while True:
#      debug()

while True:
    #print(chassis.motor_controller_1.INA1.value, chassis.motor_controller_1.INB1.value)
    if serial.in_waiting > 0:
        data_in = serial.readline()
        if data_in:
            # print(data_in.decode())
            if json.loads(data_in.decode()) == "Type":
                # print(data_in)
                # print("nav")
                serial.write(bytearray(json.dumps("nav")+"\n"))
            else:
                teleop = json.loads(data_in.decode("utf-8"))
                #print(teleop)

                # mag = math.sqrt(teleop[1]**2 + teleop[0]**2)
                # phase = math.atan2(teleop[0],teleop[1])
                # print(teleop)
                if teleop:
                    #print(teleop)
                    K_p = 35000
                    K_z = 5
                    K_l = 1.25

                    lin_x = teleop[0]
                    ang_z = teleop[1]

                    v1 = int(-K_p * ((lin_x*K_l) - ((ang_z*K_z) * (WHEEL_SEP / 2))))
                    v2 = int(-K_p * ((lin_x*K_l) + ((ang_z*K_z) * (WHEEL_SEP / 2))))
                    v3 = v1
                    v4 = v2

                    motors = [v1, v2, v3, v4]
                    #print(teleop, motors)

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


                encoder_data.update({"FR": enc_fr.get_data(time.monotonic_ns())})
                encoder_data.update({"BR": enc_br.get_data(time.monotonic_ns())})
                encoder_data.update({"BL": enc_bl.get_data(time.monotonic_ns())})
                encoder_data.update({"FL": enc_fl.get_data(time.monotonic_ns())})

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

                print(sensor_data["IMU"])
                print(sensor_data["Encoder"]["BL"]["Vel"], 
                    sensor_data["Encoder"]["FL"]["Vel"], 
                    sensor_data["Encoder"]["FR"]["Vel"],
                    sensor_data["Encoder"]["BR"]["Vel"])

                n = serial.write(remaining)
                #print(n)
                while serial.out_waiting:
                    pass
            serial.flush()
