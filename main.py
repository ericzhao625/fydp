# from bluetooth_communication import Bluetooth
# from signal import pause
from linear_actuator import LinearActuator
from aiming_motor import AimingMotor
from shooting_motor import ShootingMotor
from battery_monitor import BatteryMonitor
from imu import IMU

import board
import serial
import pigpio
import threading
from adafruit_ina3221 import INA3221
import time

# INA CHANNELS
LF_ACTUATOR_INA     = 0
LB_ACTUATOR_INA     = 1
BATTERY_MONITOR_INA = 2

# GPIO PIN NUMBERS

LF_ACTUATOR_ENABLE  = 11
I2C_SDA             = 2
I2C_SCL             = 3
LF_ACTUATOR_IN1     = 9
LF_ACTUATOR_IN2     = 10
SHOOTING_PWM        = 12
RIGHT_LIMIT_SWITCH  = 13
UART_TX             = 14
UART_RX             = 15
R_ACTUATOR_IN1      = 16
AIMING_IN2          = 17
LEFT_LIMIT_SWITCH   = 19
R_ACTUATOR_IN2      = 20
R_ACTUATOR_ENABLE   = 21
AIMING_ENABLE       = 22
LB_ACTUATOR_ENABLE  = 23
LB_ACTUATOR_IN2     = 24
LB_ACTUATOR_IN1     = 25
AIMING_IN1          = 27

class Frisbeast():

    def __init__(self):
        self.pi = pigpio.pi()

        self.i2c = board.I2C()
        self.i2c_lock = threading.Lock()

        self.ina = INA3221(self.i2c)
        self.low_battery_event = threading.Event()
        self.battery_monitor = BatteryMonitor(
            ina=self.ina,
            ina_channel=BATTERY_MONITOR_INA,
            battery_low_event=self.low_battery_event,
            i2c_lock=self.i2c_lock,
        )

        # self.uart = serial.Serial("/dev/serial0", 115200)
        self.imu = IMU(i2c=self.i2c, i2c_lock=self.i2c_lock)

        self.aiming_motor = AimingMotor(
            in1=AIMING_IN1,
            in2=AIMING_IN2,
            enable=AIMING_ENABLE,
            left_limit_switch=LEFT_LIMIT_SWITCH,
            right_limit_switch=RIGHT_LIMIT_SWITCH,
            pi=self.pi,
        )

        self.left_front_actuator = LinearActuator(
            in1=LF_ACTUATOR_IN1,
            in2=LF_ACTUATOR_IN2,
            enable=LF_ACTUATOR_ENABLE,
            ina=self.ina,
            ina_channel=LF_ACTUATOR_INA,
            i2c_lock=self.i2c_lock,
            pi=self.pi,
        )

        self.left_back_actuator = LinearActuator(
            in1=LB_ACTUATOR_IN1,
            in2=LB_ACTUATOR_IN2,
            enable=LB_ACTUATOR_ENABLE,
            ina=self.ina,
            ina_channel=LB_ACTUATOR_INA,
            i2c_lock=self.i2c_lock,
            pi=self.pi,
        )

        self.right_actuator = LinearActuator(
            in1=R_ACTUATOR_IN1,
            in2=R_ACTUATOR_IN2,
            enable=R_ACTUATOR_ENABLE,
            pi=self.pi,
        )

        self.shooting_motor = ShootingMotor(
            pwm=SHOOTING_PWM,
            pi=self.pi,
        )
    
    def shutdown(self):
        # replace below with centering before stopping
        self.aiming_motor.stop()
        # replace below with move until end before stopping
        self.left_front_actuator.stop()
        self.left_front_actuator.stop()
        self.right_actuator.stop()

        self.shooting_motor.stop()

def set_angle(goal_yaw, goal_pitch):
    # temp testing
    while True:
        yaw, pitch, _roll = frisbeast.imu.imu_readings()
        yaw = -yaw # left/right side high low
        pitch = pitch # up/back side

        yaw_error = goal_yaw - yaw
        pitch_error = goal_pitch - pitch

        if abs(yaw_error) < 0.5 and abs(pitch_error) < 0.2:
            frisbeast.left_back_actuator.stop()
            frisbeast.left_front_actuator.stop()
            print(f"goal reached, yaw: {yaw}, pitch: {pitch}")
        elif abs(yaw_error) > abs(pitch_error):
            speed = abs(yaw_error) / 20
            if yaw_error > 0:
                frisbeast.left_back_actuator.up(speed)
                frisbeast.left_front_actuator.up(speed)
            else:
                frisbeast.left_back_actuator.down(speed)
                frisbeast.left_front_actuator.down(speed)
        else:
            speed = abs(pitch_error) / 20
            if pitch_error > 0:
                frisbeast.left_front_actuator.up(speed)
                frisbeast.left_back_actuator.down(speed)
            else:
                frisbeast.left_front_actuator.down(speed)
                frisbeast.left_back_actuator.up(speed)
        
        time.sleep(0.2)
                

if __name__ == '__main__':
    frisbeast = Frisbeast()
    # bluetooth = Bluetooth()
    # try:
    #     pause()
    # except KeyboardInterrupt:
    #     print("Keyboard Interrupt detected! Cleaning up resources.")

    # finally:
    #     bluetooth.cleanup()

