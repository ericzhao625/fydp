from bluetooth_communication import Bluetooth
from signal import pause
from linear_actuator import LinearActuator, TOP, BOTTOM
from aiming_motor import AimingMotor
from shooting_motor import ShootingMotor
from battery_monitor import BatteryMonitor
# from imu import IMU
from imu_uart_rvc import IMU
from h_bridge import HBridge

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
# NO ACCESS         = 0
# NO ACCESS         = 1
I2C_SDA             = 2
I2C_SCL             = 3
IMU_RESET           = 4
AIMING_A            = 5
PUSH_MOTOR_IN1      = 6
PUSH_MOTOR_IN2      = 7
PUSH_MOTOR_ENABLE   = 8
LF_ACTUATOR_IN1     = 9
LF_ACTUATOR_IN2     = 10
LF_ACTUATOR_ENABLE  = 11
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
AIMING_B            = 26
AIMING_IN1          = 27

class Frisbeast():

    def __init__(self):
        self.goal_yaw = 0
        self.goal_pitch = 0
        self.origin = 0
        self.angle_active = threading.Event()
        self.calibrated = False
        
        self.last_activation_time = 0
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

        self.uart = serial.Serial("/dev/serial0", 115200)
        self.imu = IMU(reset_pin=IMU_RESET, uart=self.uart, pi=self.pi)
        # self.imu = IMU(i2c=self.i2c, i2c_lock=self.i2c_lock)

        self.aiming_motor = AimingMotor(
            in1=AIMING_IN1,
            in2=AIMING_IN2,
            enable=AIMING_ENABLE,
            left_limit_switch=LEFT_LIMIT_SWITCH,
            right_limit_switch=RIGHT_LIMIT_SWITCH,
            pi=self.pi,
        )

        self.push_motor = HBridge(
            in1=PUSH_MOTOR_IN1,
            in2=PUSH_MOTOR_IN2,
            enable=PUSH_MOTOR_ENABLE,
            pwm_freq=100000,
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
        set_angle_thread = threading.Thread(target=self.set_angle)
        set_angle_thread.daemon = True
        set_angle_thread.start()
    
    def stop(self):
        # replace below with centering before stopping
        self.aiming_motor.stop()
        # replace below with move until end before stopping
        self.left_front_actuator.stop()
        self.left_front_actuator.stop()
        self.right_actuator.stop()

        self.shooting_motor.stop()

    def calibrate_aiming(self):
        self.aiming_motor.left(50)
        self.aiming_motor.stopped.wait()
        left_angle, _, _ = self.imu.imu_readings()
        left_angle = (left_angle + 360) % 360
        self.aiming_motor.right(50)
        self.aiming_motor.stopped.wait()
        right_angle, _, _ = self.imu.imu_readings()
        right_angle = (right_angle + 360) % 360
        print(f"left angle: {left_angle}")
        print(f"right angle: {right_angle}")
        self.origin = left_angle

        if right_angle < left_angle:
            right_angle += 360

        middle_angle = (right_angle + left_angle) / 2
        self.calibrated = True

        self.turn_to_angle(90)
        # while True:
        #     angle, _, _ = self.imu.imu_readings()
        #     angle = (angle + 360) % 360
            
        #     error = middle_angle - angle
        #     if abs(error) <= 0.5:
        #         self.aiming_motor.stop()
        #         return
        #     elif error > 0.5:
        #         self.aiming_motor.right(error)
        #     else:
        #         self.aiming_motor.left(-error)

    def turn_to_angle(self, goal):
        
        if not self.calibrated:
            self.calibrate()
        goal = self.origin + goal
        print(f"TURNING TO: {goal}")
        stable_start_time = time.time()
        stable = False
        previous_error = 0
        prev_time = time.time()

        while True:
            # print("looping")
            angle, _, _ = self.imu.imu_readings()
            if angle is None:
                continue
            angle = (angle + 360) % 360
            if angle < self.origin:
                angle += 360

            error = goal - angle
            P = 1.1
            # D = 1
            # derivative = (error - previous_error) / (time.time() - prev_time)
            if abs(error) <= 2:
                self.aiming_motor.stop()
                time.sleep(0.1)
                return
                # if not stable:
                #     stable = True
                #     stable_start_time = time.time()
                #     self.aiming_motor.stop()
                #     time.sleep(0.1)
                # elif time.time() - stable_start_time > 0.2:
                #     return
            speed = P * error
            # previous_error = error
            if error > 0:
                # stable = False
                print(f"going right {speed}")
                self.aiming_motor.right(abs(speed))
            else:
                print(f"going left {error}")
                # stable = False
                self.aiming_motor.left(abs(speed))
            time.sleep(0.02)
    
    def fix_angle(self, error):
        print(f"ERROR: {error}")
        if not self.calibrated:
            self.calibrate()

        angle, _, _ = self.imu.imu_readings()
        if angle is None:
            return
        angle = (angle + 360) % 360
        if angle < self.origin:
            angle += 360
        
        print(f"CURRENT ANGLE: {angle}")

        self.turn_to_angle(angle + error - self.origin)
    

    def calibrate(self):
        print("Starting calibration...")
        calibrate_aiming_thread = threading.Thread(target=self.calibrate_aiming)
        calibrate_aiming_thread.start()
        threads = [
            self.right_actuator.calibrate(wait=False),
            self.left_back_actuator.calibrate(wait=False),
            self.left_front_actuator.calibrate(wait=False),
            calibrate_aiming_thread,
        ]

        for thread in threads:
            thread.join()
        print("Finished calibration")
        self.calibrated = True
    
    def push_frisbee(self, distance, pose_estimation):
        """
        Activates the solenoid to push the frisbee if the correct pose is detected.

        Args:
            pose_estimation (str): The detected pose state from CV.
        """
        
        current_time = time.time()
        if pose_estimation in ('centered and throw identified', 'Direction:Throw') and distance >= 5:
            if current_time - self.last_activation_time >= 5:
                print("Solenoid Activated")
                self.push_motor.forward(100)
                time.sleep(0.3)
                self.push_motor.stop()

                self.last_activation_time = current_time

            else:
                print("Cooldown active, solenoid not triggered.")
        elif pose_estimation == 'centered and throw identified' and distance < 5:
            print('Too close')

    def activate_angle(self):
        self.angle_active.set()
    
    def deactivate_angle(self):
        self.angle_active.clear()

    def set_angle(self):
        # temp testing
        # TODO fix the cases
        # if goal_yaw < -1:
        #     frisbeast.right_actuator.move_to_top(False)
        # elif goal_yaw > 1:
        #     frisbeast.right_actuator.move_to_bottom(False)
        # else:
        #     frisbeast.right_actuator.move_to_middle(False)

        while True:
            self.angle_active.wait()
            if self.goal_yaw < -1:
                frisbeast.right_actuator.move_to_top(False)
            elif self.goal_yaw > 1:
                frisbeast.right_actuator.move_to_bottom(False)
            else:
                if self.left_back_actuator.state == TOP or self.left_front_actuator.state == TOP:
                    self.right_actuator.down(100)
                elif self.left_back_actuator.state == BOTTOM or self.left_front_actuator.state == BOTTOM:
                    self.right_actuator.up(100)
                else:
                    self.right_actuator.stop()

            roll, pitch, yaw = self.imu.imu_readings()
            if yaw is None or pitch is None or roll is None:
                continue
            # positive yaw: right lower than left
            # negative yaw: right higher than left
            # yaw = -yaw # left/right side high low
            middle_yaw = -178.27
            if yaw > 0:
                yaw = 360 + middle_yaw - yaw
            elif middle_yaw < yaw < 0:
                yaw = middle_yaw - yaw
            else:
                yaw = -yaw + middle_yaw
            
            pitch = - pitch # up/back side

            yaw_error = self.goal_yaw - yaw
            pitch_error = self.goal_pitch - pitch

            if abs(yaw_error) < 0.25 and abs(pitch_error) < 0.25:
                self.left_back_actuator.stop()
                self.left_front_actuator.stop()
            # elif abs(yaw_error) > abs(pitch_error):
            elif abs(pitch_error) > abs(yaw_error):
            # elif abs(pitch_error) > 0.33:
                # print("correcting pitch")
                speed = abs(pitch_error) * 5
                if pitch_error > 0:
                    self.left_front_actuator.up(speed)
                    self.left_back_actuator.down(speed)
                else:
                    self.left_front_actuator.down(speed)
                    self.left_back_actuator.up(speed)
            else:
                # print("correcting yaw")
                speed = abs(yaw_error) * 5
                if yaw_error > 0:
                    self.left_back_actuator.up(speed)
                    self.left_front_actuator.up(speed)
                else:
                    self.left_back_actuator.down(speed)
                    self.left_front_actuator.down(speed)
            
            # print(f"yaw: {yaw} goal: {self.goal_yaw}, pitch: {pitch} goal: {self.goal_pitch}")
            time.sleep(0.03)
                

if __name__ == '__main__':
    frisbeast = Frisbeast()
    bluetooth = Bluetooth(frisbeast)
    try:
        pause()
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected! Cleaning up resources.")

    finally:
        bluetooth.cleanup()
