import RPi.GPIO as GPIO
import constants

class Throw:
    def __init__(self):
        self.min_distance = constants.MIN_DISTANCE
        self.max_distance = constants.MAX_DISTANCE
        self.min_pwm = constants.MIN_PWM
        self.max_pwm = constants.MAX_PWM
        self.pwm_pin = constants.PWM_PIN
        self.solenoid_pin = constants.SOLENOID_PIN

        # GPIO initialization
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, constants.PWM_FREQ)
        self.pwm.start(constants.PWM_INIT_DC)

        GPIO.setup(self.solenoid_pin, GPIO.OUT)
        GPIO.output(self.solenoid_pin, GPIO.LOW)


    def distance_to_pwm(self, distance):
        """
        Calculate PWM duty cycle based on distance obtained from CV

        :param distance: distance
        :return: PWM duty cycle
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
        pwm_value = self.distance_to_pwm(distance)
        self.pwm.ChangeDutyCycle(pwm_value)
        if distance and pwm_value:
            print(f"Distance: {distance:.2f}m, PWM: {distance:.2f}%")

        return pwm_value

    def stop_motor(self):
        print('Stopping throwing motor')

        try:
            self.pwm.stop()
        except RuntimeError as e:
            print(f"Warning: GPIO cleanup failed - {e}")
        except Exception as e:
            print(f"Unexpected error during cleanup: {e}")
            
        print('Throwing motor stopped')
