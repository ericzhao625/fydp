import pigpio
import atexit

FREQ = 125
MIN_SPEED = 0
MAX_SPEED = 100

class ShootingMotor:
    """
    A class to control an H-Bridge motor driver using Raspberry Pi GPIO.

    This class provides methods to drive a motor forward, backward, stop it,
    and safely clean up GPIO resources.
    
    Attributes:
        in1 (int): GPIO pin for motor direction control (IN1).
        in2 (int): GPIO pin for motor direction control (IN2).
        enable (int): GPIO pin for PWM control (EN).
        pwm_freq (int): Frequency of the PWM signal.
        min_duty_cycle (float): Minimum duty cycle allowed,
        max_duty_cycle (float): Maximum duty cycle allowed,
        pi: pigpio.pi() object
        pwm_range (int): Resolution of duty cycle
    """

    def __init__(
        self,
        pwm: int,
        pwm_freq: int=FREQ,
        min_speed: float=MIN_SPEED,
        max_speed: float=MAX_SPEED,
        pi=None,
        pwm_range: int=1023,
    ):
        """
        Initializes the H-Bridge motor driver and sets up GPIO pins.

        Args:
            
            in1 (int): GPIO pin for motor direction (IN1).
            in2 (int): GPIO pin for motor direction (IN2).
            enable (int): GPIO pin for enabling the motor (PWM pin).
            pwm_freq (int): Frequency of the PWM signal in Hz.
            min_duty_cycle (float): Minimum duty cycle for movement.
            max_duty_cycle (float): Maximum duty cycle allowed.
            pi: pigpio.pi() object.
            pwm_range (int): Resolution of pwm value (25-40000)
        """
        self.pwm = pwm
        self.pwm_freq = pwm_freq
        self.pwm_range = pwm_range

        self.min_speed = min_speed
        self.speed_range = max_speed - min_speed

        self.period_us = 1 / pwm_freq * 1e6
        self.NEUTRAL_US = 1500

        self.REVERSE_MAX_US = 1000
        self.REVERSE_MIN_US = 1475
        self.REVERSE_RANGE_US = self.REVERSE_MIN_US - self.REVERSE_MAX_US

        self.FORWARD_MAX_US = 2000
        self.FORWARD_MIN_US = 1525
        self.FORWARD_RANGE_US = self.FORWARD_MAX_US - self.FORWARD_MIN_US

        self.pwm_dc = self.NEUTRAL_US

        if not pi:
            self.pi = pigpio.pi()
        else:
            self.pi = pi

        self.pi.set_mode(pwm, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(pwm, pwm_freq)
        self.pi.set_PWM_range(pwm, pwm_range)
        self.stop()

        atexit.register(self.stop)
    
    def set_dutycycle(self, pulse_width_us):
        pwm_dc = round(pulse_width_us / self.period_us * self.pwm_range)
        
        if pwm_dc != self.pwm_dc:
            self.pi.set_PWM_dutycycle(self.pwm, pwm_dc)
            self.pwm_dc = pwm_dc

    def forward(self, speed: float):
        """
        Drives the motor forward by setting IN1 high and IN2 low.

        Args:
            speed (float): Speed in percent
        """
        speed = speed / 100 * self.speed_range + self.min_speed
        pulse_width_us = speed / 100 * self.FORWARD_RANGE_US + self.FORWARD_MIN_US
        self.set_dutycycle(pulse_width_us)
    
    def stop(self):
        self.set_dutycycle(self.NEUTRAL_US)

    ### DO NOT SPIN IN REVERSE WILL THROW FRISBEE IN WRONG DIRECTION
    # def reverse(self, speed):
    #     """
    #     Drives the motor backward by setting IN1 low and IN2 high.
        
    #     Args:
    #         Speed (float): Motor speed, 0-100%
    #     """
    #     speed = speed / 100 * self.speed_range + self.min_speed
    #     # Reverse max speed is lower period than min speed
    #     pulse_width_us = speed / 100 * self.REVERSE_RANGE_US + self.REVERSE_MAX_US
    #     self.set_dutycycle(pulse_width_us)
