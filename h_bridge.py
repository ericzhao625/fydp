import pigpio
import atexit

FORWARD = 1
STOPPED = 0
REVERSE = -1

class HBridge:
    """
    A class to control an H-Bridge motor driver using Raspberry Pi GPIO.

    This class provides methods to drive a motor forward, reverse, stop it,
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
        in1: int,
        in2: int,
        enable: int,
        pwm_freq: int,
        min_duty_cycle: float=0,
        max_duty_cycle: float=100,
        pi=None,
        pwm_range: int=511,
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
        self.in1 = in1
        self.in2 = in2
        self.enable = enable
        self.pwm_freq = pwm_freq
        self.pwm_range = pwm_range

        self.min_duty_cycle = min_duty_cycle / 100 * pwm_range
        self.duty_cycle_range = (max_duty_cycle - min_duty_cycle) / 100 * pwm_range

        self.pwm_dc = 0
        self.direction = STOPPED

        if not pi:
            self.pi = pigpio.pi()
        else:
            self.pi = pi

        self.pi.set_mode(enable, pigpio.OUTPUT)
        self.pi.set_mode(in1, pigpio.OUTPUT)
        self.pi.set_mode(in2, pigpio.OUTPUT)

        self.pi.set_PWM_frequency(enable, pwm_freq)
        self.pi.set_PWM_range(enable, pwm_range)

        atexit.register(self.stop)

    def set_duty_cycle(self, speed: float):
        duty_cycle = round((speed / 100) * self.duty_cycle_range + self.min_duty_cycle)
        # print(f"Setting duty cycle to {duty_cycle} / {self.pwm_range} ({duty_cycle} / {self.pwm_range})")
        if duty_cycle != self.pwm_dc:
            self.pi.set_PWM_dutycycle(self.enable, duty_cycle)
            self.pwm_dc = duty_cycle

    def forward(self, speed: float):
        """
        Drives the motor forward by setting IN1 high and IN2 low.

        Args:
            speed (float): Speed in percent
        """
        self.set_duty_cycle(speed)
        
        if self.direction != FORWARD:
            self.pi.write(self.in2, 0)
            self.pi.write(self.in1, 1)
            self.direction = FORWARD

    def reverse(self, speed):
        """
        Drives the motor reverse by setting IN1 low and IN2 high.
        
        Args:
            pwm (float): Motor PWM value
        """
        self.set_duty_cycle(speed)

        if self.direction != REVERSE:
            self.pi.write(self.in1, 0)
            self.pi.write(self.in2, 1)
            self.direction = REVERSE
        
    def stop(self):
        """
        Stops the motor by setting both IN1 and IN2 low.
        """
        self.set_duty_cycle(0)

        if self.direction != STOPPED:
            self.direction = STOPPED
            self.pi.write(self.in1, 0)
            self.pi.write(self.in2, 0)
