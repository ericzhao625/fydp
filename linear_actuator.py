import constants
from h_bridge import HBridge


class LinearActuator(HBridge):
    """
    A class to control the tilting mechanism using an H-Bridge motor driver.

    This class extends HBridge to control a motor that adjusts tilt.

    Attributes:
        (Inherited from HBridge)
    """
    def __init__(
        self,
        in1,
        in2,
        enable,
        pwm_freq=constants.LINEAR_ACTUATOR_FREQ,
        pwm_dc=constants.LINEAR_ACTUATOR_DC
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
        super().__init__(in1, in2, enable, pwm_freq, pwm_dc)
