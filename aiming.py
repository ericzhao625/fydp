import constants
from h_bridge import HBridge
from pid import PIDController


class Aim(HBridge):
    """
    A class to control the aiming mechanism using an H-Bridge motor driver.

    This class extends HBridge to control a motor that adjusts aiming based on 
    pose estimation inputs.

    Attributes:
        (Inherited from HBridge)
    """
    def __init__(
        self,
        in1=constants.IN7,
        in2=constants.IN8,
        enable=constants.ENABLE_D,
        pwm_freq=constants.AIMING_MOTOR_FREQ,
        pwm_dc=constants.AIMING_MOTOR_DC
    ):
        """
        Initializes the Aim control system by setting up the H-Bridge motor.

        Args:
            in1 (int): GPIO pin for motor direction (default: constants.IN7).
            in2 (int): GPIO pin for motor direction (default: constants.IN8).
            enable (int): GPIO pin for enabling the motor (default: constants.ENABLE_D).
            pwm_freq (int): Frequency of the PWM signal in Hz (default: constants.AIMING_MOTOR_FREQ).
            pwm_dc (int): Initial duty cycle (0-100%) (default: constants.AIMING_MOTOR_DC).
        """
        super().__init__(in1, in2, enable, pwm_freq, pwm_dc)
        self.deadband = constants.AIMING_DEADBAND
        self.pid_controller = PIDController()

    
    def track_player(self, angle):
        """
        Adjusts the aiming direction based on the player's movement.

        Args:
            angle (float): Angle deviation from the target.
        """
        # Check angle is valid
        if angle is None:
            return

        # Ignore small angle deviations
        if abs(angle) < self.deadband:
            self.stop()
        
        # Compute PWM
        else:
            pwm = self.pid_controller.compute(angle)
            if angle > 0:
                self.forward(pwm)
            else:
                self.backward(pwm)

    def turn(self, direction):
        """
        Manual operation of aiming based on App input.

        Args:
            direction (string): command from app.
        """
        if direction == 'left':
            self.forward(self.pwm_dc)
        
        elif direction == 'right':
            self.backward(self.pwm_dc)
        
        else:
            self.stop()
