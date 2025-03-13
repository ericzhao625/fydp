import constants
from h_bridge import HBridge
from aiming_motor import AimingMotor
from pid import PIDController

FREQ = 10000
MIN_DUTY_CYCLE = 70
MAX_DUTY_CYCLE = 100
RESET_LIMIT_SWITCH_SPEED = 25

OK = 0
TOO_FAR_LEFT = 1
TOO_FAR_RIGHT = 2

class Aim(AimingMotor):
    """
    A class to control the aiming mechanism using an H-Bridge motor driver.

    This class extends HBridge to control a motor that adjusts aiming based on 
    pose estimation inputs.

    Attributes:
        (Inherited from HBridge)
    """
    def __init__(
        self,
        in1: int=27,
        in2: int=17,
        enable: int=22,
        left_limit_switch: int=19,
        right_limit_switch: int=13,
        pwm_freq: int=FREQ,
        min_duty_cycle: float=MIN_DUTY_CYCLE,
        max_duty_cycle: float=MAX_DUTY_CYCLE,
        pi=None,
        pwm_range: int=511,
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
        super().__init__() # TEMP
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
            if angle < 0:
                pwm = self.pid_controller.compute(angle + self.deadband)
            else:
                pwm = self.pid_controller.compute(angle - self.deadband)
            if pwm is not None:
                if angle > 0:
                    self.right(pwm)
                else:
                    self.left(pwm)

    def turn(self, direction):
        """
        Manual operation of aiming based on App input.

        Args:
            direction (string): command from app.
        """
        print(f"direction: {direction}")
        if direction == 'Direction:Left':
            self.left(50)
        
        elif direction == 'Direction:Right':
            self.right(50)
        
        else:
            self.stop()
            print('No turn')
