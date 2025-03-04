import RPi.GPIO as GPIO
import time

import constants


class Throw:
    """
    Controls the frisbee throwing mechanism.

    This class manages motor speed based on estimated distance and triggers a solenoid
    to push the frisbee when the correct pose is detected.

    Attributes:
        pwm_pin (int): GPIO pin for PWM control of the motor.
        min_distance (float): Minimum distance threshold for scaling PWM.
        max_distance (float): Maximum distance threshold for scaling PWM.
        min_pwm (float): Minimum PWM duty cycle percentage.
        max_pwm (float): Maximum PWM duty cycle percentage.
        solenoid_pin (int): GPIO pin controlling the solenoid.
        cool_down (float): Time in seconds before solenoid can be activated again.
        last_activation_time (float): Timestamp of the last solenoid activation.

    Methods:
        distance_to_pwm(distance): Converts distance to PWM duty cycle.
        update_motor_speed(distance): Updates motor speed based on distance.
        push_frisbee(pose_estimation): Activates solenoid if throwing conditions are met.
        stop_motor(): Stops the throwing motor and handles cleanup.
    """

    def __init__(self):
        """
        Initializes the Throw class with GPIO setup for motor and solenoid control.
        """
        self.pwm_pin = constants.PWM_PIN
        self.min_distance = constants.MIN_DISTANCE
        self.max_distance = constants.MAX_DISTANCE
        self.min_pwm = constants.MIN_PWM
        self.max_pwm = constants.MAX_PWM

        self.solenoid_pin = constants.SOLENOID_PIN
        self.cool_down = constants.COOL_DOWN
        self.last_activation_time = 0

        # GPIO initialization
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, constants.PWM_FREQ)
        self.pwm.start(constants.PWM_INIT_DC)

        GPIO.setup(self.solenoid_pin, GPIO.OUT)
        GPIO.output(self.solenoid_pin, GPIO.LOW)


    def distance_to_pwm(self, distance):
        """
        Converts estimated distance to an appropriate PWM duty cycle.

        Args:
            distance (float): The estimated distance from CV in meters.

        Returns:
            float: PWM duty cycle percentage based on the distance.
        """
        if distance:
            if distance < self.min_distance:
                pwm_value = 0
            elif distance > self.max_distance:
                pwm_value = self.max_pwm
            else:
                pwm_value = self.min_pwm + (self.max_pwm - self.min_pwm) * ((distance - self.min_distance) / (self.max_distance - self.min_distance))
            return pwm_value
        return 0

    def update_motor_speed(self, distance):
        """
        Updates the motor speed based on the estimated distance.

        Args:
            distance (float): The estimated distance in meters.

        Returns:
            float: The calculated PWM duty cycle value.
        """
        pwm_value = self.distance_to_pwm(distance)
        self.pwm.ChangeDutyCycle(pwm_value)
        if distance and pwm_value:
            print(f"Distance: {distance:.2f}m, PWM: {pwm_value:.2f}%")

        return pwm_value

    def push_frisbee(self, distance, pose_estimation):
        """
        Activates the solenoid to push the frisbee if the correct pose is detected.

        Args:
            pose_estimation (str): The detected pose state from CV.
        """
        current_time = time.time()
        if pose_estimation == 'centered and throw identified' and distance >= self.min_distance:
            if current_time - self.last_activation_time >= self.cool_down:
                print("Solenoid Activated")
                GPIO.output(self.solenoid_pin, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(self.solenoid_pin, GPIO.LOW)

                self.last_activation_time = current_time

            else:
                print("Cooldown active, solenoid not triggered.")
        elif pose_estimation == 'centered and throw identified' and distance < self.min_distance:
            print('Too close')

    def stop_motor(self):
        """
        Stops the throwing motor.
        """
        print('Stopping throwing motor')

        try:
            self.pwm.stop()
        except RuntimeError as e:
            print(f"Warning: GPIO cleanup failed - {e}")
        except Exception as e:
            print(f"Unexpected error during cleanup: {e}")

        print('Throwing motor stopped')
