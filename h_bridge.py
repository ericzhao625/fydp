import RPi.GPIO as GPIO          
from time import sleep

class HBridge:
    """
    A class to control an H-Bridge motor driver using Raspberry Pi GPIO.

    This class provides methods to drive a motor forward, backward, stop it,
    and safely clean up GPIO resources.
    
    Attributes:
        pin1 (int): GPIO pin for motor direction control (IN1).
        pin2 (int): GPIO pin for motor direction control (IN2).
        enable (int): GPIO pin for PWM control (EN).
        pwm_freq (int): Frequency of the PWM signal.
        pwm_dc (int): Initial duty cycle of the PWM signal.
        pwm (GPIO.PWM): PWM control object.
    """
    def __init__(self, in1, in2, enable, pwm_freq, pwm_dc):
        """
        Initializes the H-Bridge motor driver and sets up GPIO pins.

        Args:
            in1 (int): GPIO pin for motor direction (IN1).
            in2 (int): GPIO pin for motor direction (IN2).
            enable (int): GPIO pin for enabling the motor (PWM pin).
            pwm_freq (int): Frequency of the PWM signal in Hz.
            pwm_dc (int): Initial duty cycle (0-100%).
        """
        self.pin1 = in1
        self.pin2 = in2
        self.enable = enable
        self.pwm_freq = pwm_freq
        self.pwm_dc = pwm_dc

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin1,GPIO.OUT)
        GPIO.setup(self.pin2,GPIO.OUT)
        GPIO.setup(self.enable,GPIO.OUT)
        GPIO.output(self.pin1,GPIO.LOW)
        GPIO.output(self.pin2,GPIO.LOW)
        self.pwm = GPIO.PWM(self.enable, self.pwm_freq)
        self.pwm.start(self.pwm_dc)

    def forward(self):
        """
        Drives the motor forward by setting IN1 high and IN2 low.
        """
        GPIO.output(self.pin1, GPIO.HIGH)
        GPIO.output(self.pin2, GPIO.LOW)

    def backward(self):
        """
        Drives the motor backward by setting IN1 low and IN2 high.
        """
        GPIO.output(self.pin1,GPIO.LOW)
        GPIO.output(self.pin2,GPIO.HIGH)
        
    def stop(self):
        """
        Stops the motor by setting both IN1 and IN2 low.
        """
        GPIO.output(self.pin1,GPIO.LOW)
        GPIO.output(self.pin2,GPIO.LOW)

    def stop_h_bridge(self):
        """
        Stops PWM.

        This method stops the PWM signal and handles any cleanup errors gracefully.

        Raises:
            RuntimeError: If GPIO cleanup fails.
            Exception: If an unexpected error occurs.
        """
        print('Stopping h bridge control')

        try:
            self.pwm.stop()
        except RuntimeError as e:
            print(f"Warning: GPIO cleanup failed - {e}")
        except Exception as e:
            print(f"Unexpected error during cleanup: {e}")

        print('H bridge control stopped')
