from h_bridge import HBridge

FREQ = 10000
MIN_DUTY_CYCLE = 80
MAX_DUTY_CYCLE = 100

class LinearActuator(HBridge):
    """
    A class to control the tilting mechanism using an H-Bridge motor driver.

    This class extends HBridge to control a motor that adjusts tilt.

    Attributes:
        (Inherited from HBridge)
    """
    def __init__(
        self,
        in1: int,
        in2: int,
        enable: int,
        pwm_freq: int=FREQ,
        min_duty_cycle: float=MIN_DUTY_CYCLE,
        max_duty_cycle: float=MAX_DUTY_CYCLE,
        pi=None,
        pwm_range: int=511,
    ):
        """
        Initializes the linear actuator control system by setting up the H-Bridge motor.

        Args:
            in1 (int): GPIO pin for motor direction.
            in2 (int): GPIO pin for motor direction.
            enable (int): GPIO pin for enabling the motor.
            pwm_freq (int): Frequency of the PWM signal in Hz (default: constants.LINEAR_ACTUATOR_FREQ).
            pwm_dc (int): Initial duty cycle (0-100%) (default: constants.LINEAR_ACTUATOR_DC).
        """
        super().__init__(in1, in2, enable, pwm_freq, min_duty_cycle, max_duty_cycle, pi, pwm_range)

    def up(self, speed):
        self.forward(speed)

    def down(self, speed):
        self.backward(speed)
