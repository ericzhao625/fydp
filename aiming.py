from h_bridge import HBridge
import constants

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
    
    def track_player(self, pose_estimation):
        """
        Adjusts the aiming direction based on the player's movement.

        Args:
            pose_estimation (str): The detected pose direction.
                Expected values:
                - "move left" -> Motor moves forward
                - "move right" -> Motor moves backward
                - Any other value -> Motor stops
        """
        if pose_estimation == 'move left':
            self.forward()
        elif pose_estimation == 'move right':
            self.backward()
        else:
            self.stop()