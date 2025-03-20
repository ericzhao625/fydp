from bluetooth_communication import Bluetooth
from signal import pause
from linear_actuator import LinearActuator, TOP, BOTTOM
from aiming_motor import AimingMotor
from shooting_motor import ShootingMotor
from battery_monitor import BatteryMonitor
# from imu import IMU
from imu_uart_rvc import IMU
from h_bridge import HBridge, FORWARD, STOPPED, REVERSE

import board
import serial
import pigpio
import threading
from adafruit_ina3221 import INA3221
import time
import math

# INA CHANNELS
LF_ACTUATOR_INA     = 0
LB_ACTUATOR_INA     = 1
BATTERY_MONITOR_INA = 2

# GPIO PIN NUMBERS
# NO_ACCESS_0         = 0
# NO_ACCESS_1         = 1
# I2C_SDA             = 2 # default HIGH
# I2C_SCL             = 3 # default HIGH
IMU_RESET           = 4 # default HIGH
AIMING_A            = 5 # default HIGH
PUSH_MOTOR_IN1      = 6 # default HIGH
PUSH_MOTOR_IN2      = 7 # default HIGH
PUSH_MOTOR_A        = 8 # default HIGH
LF_ACTUATOR_IN1     = 9
LF_ACTUATOR_IN2     = 10
LF_ACTUATOR_ENABLE  = 11
SHOOTING_PWM        = 12
RIGHT_LIMIT_SWITCH  = 13
# UART_TX             = 14
# UART_RX             = 15
R_ACTUATOR_IN1      = 16
AIMING_IN2          = 17
PUSH_MOTOR_ENABLE   = 18
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
        self.goal_pitch = 0
        self.goal_roll = 0
        self.goal_yaw = 0
        self.origin = 0
        self.angle_active = threading.Event()
        self.yaw_active = threading.Event()
        self.yaw_stable = threading.Event()
        self.calibrated = threading.Event()
        self.calibrating = threading.Event()
        self.level = threading.Event()
        self.hand_raised = False

        self.AIM_P_CALIBRATION = 1
        self.AIM_D_CALIBRATION = 1
        self.AIM_P_NORM = 3.5
        self.AIM_D_NORM = 5
        self.AIM_P = self.AIM_P_CALIBRATION
        self.AIM_D = self.AIM_D_CALIBRATION
        
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
            max_duty_cycle=47.2,
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
        level_control_thread = threading.Thread(target=self.set_level)
        level_control_thread.daemon = True
        level_control_thread.start()

        yaw_control_thread = threading.Thread(target=self.yaw_controller)
        yaw_control_thread.daemon = True
        yaw_control_thread.start()
    
    def stop(self):
        self.angle_active.clear()
        # replace below with centering before stopping
        self.aiming_motor.stop()
        # replace below with move until end before stopping
        self.left_front_actuator.stop()
        self.left_front_actuator.stop()
        self.right_actuator.stop()

        self.shooting_motor.stop()

    def calibrate_aiming(self):
        self.aiming_motor.left(30)
        self.aiming_motor.stopped.wait()
        left_angle = self.imu.yaw
        left_angle = (left_angle + 360) % 360
        self.aiming_motor.right(30)
        self.aiming_motor.stopped.wait()
        right_angle = self.imu.yaw
        right_angle = (right_angle + 360) % 360
        print(f"left angle: {left_angle}")
        print(f"right angle: {right_angle}")
        self.origin = left_angle
        print(f"origin: {self.origin}")

        if right_angle < left_angle:
            right_angle += 360

        self.middle_angle = (right_angle - left_angle) / 2
        print(f"middle_angle: {self.middle_angle}")
        # self.calibrated.set()

        self.goal_yaw = self.middle_angle
        self.AIM_P = self.AIM_P_CALIBRATION
        self.AIM_D = self.AIM_D_CALIBRATION
        self.yaw_active.set()
        # self.turn_to_angle()

    def yaw_controller(self):
        stable_start_time = time.time()
        stable = False
        previous_error = 0
        prev_time = time.time()
        
        # P = 5
        # D = 8
        STABLE_THRESHOLD = 3
        UNSTABLE_THRESHOLD = 1
        threshold = STABLE_THRESHOLD

        while True:
            self.yaw_active.wait()
            self.imu.new_data.wait()
            goal = self.origin + self.goal_yaw

            # BELOW IS TO LIMIT THE TRAVEL FOR SYMPOSIUM
            # if abs(goal - (self.middle_angle + self.origin)) > 60:
            #     if goal > (self.middle_angle + self.origin):
            #         goal = self.middle_angle + self.origin + 60
            #     else:
            #         goal = self.middle_angle + self.origin - 60
            # NEED SOMETHING TO FIX STROBING

            angle = self.imu.yaw
            if angle is None:
                continue

            angle = (angle + 360) % 360
            if angle < self.origin:
                angle += 360
    
            error = goal - angle
            
            derivative = (error - previous_error) / (time.time() - prev_time)
            
            speed = abs(self.AIM_P * error + self.AIM_D * derivative)
            previous_error = error

            if abs(error) <= threshold:
                self.aiming_motor.stop()
                if not stable:
                    stable = True
                    threshold = UNSTABLE_THRESHOLD
                    stable_start_time = time.time()
                elif time.time() - stable_start_time > 0.5:
                    self.yaw_stable.set()
                    threshold = UNSTABLE_THRESHOLD
            else:
                stable = False
                threshold = STABLE_THRESHOLD
                self.yaw_stable.clear()
                if error > 0:
                    self.aiming_motor.right(speed)
                else:
                    self.aiming_motor.left(speed)
    
    def fix_angle(self, error, timestamp):
        if error is None:
            if not self.aiming_motor.strobing_mode.is_set() or self.yaw_active.is_set():
                print("yaw_active clear")
                self.yaw_active.clear()
                self.aiming_motor.strobing_mode.set()
                # self.aiming_motor.strobing_mode = True
                # print(self.aiming_motor.strobing_mode)
            
            # time.sleep(0.1)
            if self.aiming_motor.direction == FORWARD:
                print("tryna forward")
                self.aiming_motor.right(28)
            else:
                print('tryna reverse')
                self.aiming_motor.left(28)
            
            return
        
        if self.aiming_motor.strobing_mode.is_set() or not self.yaw_active.is_set():
            print("yaw_active set")
            # self.aiming_motor.strobing_mode = False
            self.aiming_motor.strobing_mode.clear()
            self.yaw_active.set()

        index = round((time.time() - timestamp) * 100)

        angle = (self.imu.yaw_deque[index] + 360) % 360

        if angle < self.origin:
            angle += 360
        
        self.goal_yaw = angle + error - self.origin
    

    def calibrate(self):
        if self.calibrating.is_set():
            return self.calibrated.wait()

        self.calibrating.set()

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
        
        self.yaw_stable.wait()
        self.AIM_P = self.AIM_P_NORM
        self.AIM_D = self.AIM_D_NORM
        print("Finished calibration")
        self.calibrated.set()
        self.calibrating.clear()

    def home(self):
        home_angle_thread = threading.Thread(target=self.set_level, args=(self.middle_angle,))
        home_angle_thread.start()
        threads = [
            self.right_actuator.move_to_bottom(wait=False),
            self.left_back_actuator.move_to_bottom(wait=False),
            self.left_front_actuator.move_to_bottom(wait=False),
            home_angle_thread,
        ]
        for thread in threads:
            thread.join()

        print("Finished calibration")
    
    def push_frisbee(self, distance, pose_estimation):
        """
        Activates the solenoid to push the frisbee if the correct pose is detected.

        Args:
            pose_estimation (str): The detected pose state from CV.
        """
        
        current_time = time.time()
        if pose_estimation in ('centered and throw identified', 'Direction:Throw') and 7.5 >= distance >= 3.5 and self.level.is_set():
            if current_time - self.last_activation_time >= 5 and self.hand_raised:
                print("Solenoid Activated")
                self.push_motor.forward(100)
                time.sleep(0.33)
                self.push_motor.stop()

                self.last_activation_time = current_time
                self.hand_raised = False
            elif current_time - self.last_activation_time < 5:
                print("Cooldown active, solenoid not triggered.")
                self.hand_raised = False
            else:
                self.hand_raised = True
        elif pose_estimation == 'centered and throw identified':
            self.hand_raised = False
            print('Too close')
        else:
            self.hand_raised = False

    def activate_angle(self):
        self.angle_active.set()
    
    def deactivate_angle(self):
        self.angle_active.clear()

    def set_level(self):
        # temp testing
        # TODO fix the cases
        # if goal_pitch < -1:
        #     frisbeast.right_actuator.move_to_top(False)
        # elif goal_pitch > 1:
        #     frisbeast.right_actuator.move_to_bottom(False)
        # else:
        #     frisbeast.right_actuator.move_to_middle(False)
        last_level_time = None

        while True:
            self.angle_active.wait()
            self.imu.new_data.wait()
            if self.goal_pitch < -1:
                frisbeast.right_actuator.move_to_top(False)
            elif self.goal_pitch > 1:
                frisbeast.right_actuator.move_to_bottom(False)
            else:
                if self.left_back_actuator.state == TOP or self.left_front_actuator.state == TOP:
                    self.right_actuator.down(100)
                elif self.left_back_actuator.state == BOTTOM or self.left_front_actuator.state == BOTTOM:
                    self.right_actuator.up(100)
                else:
                    self.right_actuator.stop()

            roll = self.imu.roll
            pitch = self.imu.pitch
            if pitch is None or roll is None:
                continue
            # positive pitch: right lower than left
            # negative pitch: right higher than left
            middle_pitch = -179.2
            middle_roll = 0.68
            if pitch > 0:
                pitch = 360 + middle_pitch - pitch
            elif middle_pitch < pitch < 0:
                pitch = middle_pitch - pitch
            else:
                pitch = -pitch + middle_pitch
            
            roll = middle_roll - roll # up/back side

            pitch_error = self.goal_pitch - pitch
            roll_error = self.goal_roll - roll

            if abs(pitch_error) < 0.4 and abs(roll_error) < 0.4:
                self.left_back_actuator.stop()
                self.left_front_actuator.stop()
                self.level.set()
                last_level_time = time.time()
            else:
                if self.level.is_set() and time.time() - last_level_time > 0.5:
                    self.level.clear()
    
                if abs(roll_error) > abs(pitch_error):
                    speed = abs(roll_error) * 5
                    if roll_error > 0:
                        self.left_front_actuator.up(speed)
                        self.left_back_actuator.down(speed)
                    else:
                        self.left_front_actuator.down(speed)
                        self.left_back_actuator.up(speed)
                else:
                    speed = abs(pitch_error) * 5
                    if pitch_error > 0:
                        self.left_back_actuator.up(speed)
                        self.left_front_actuator.up(speed)
                    else:
                        self.left_back_actuator.down(speed)
                        self.left_front_actuator.down(speed)

if __name__ == '__main__':
    frisbeast = Frisbeast()
    bluetooth = Bluetooth(frisbeast)
    while True:
        try:
            pause()
        except KeyboardInterrupt:
            print("Keyboard Interrupt detected! Cleaning up resources.")
            break

    # finally:
    bluetooth.cleanup()
