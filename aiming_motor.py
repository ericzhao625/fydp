from h_bridge import HBridge
import pigpio
import threading
import time

FREQ = 10000
MIN_DUTY_CYCLE = 60
MAX_DUTY_CYCLE = 100
RESET_LIMIT_SWITCH_SPEED = 25

OK = 0
TOO_FAR_LEFT = 1
TOO_FAR_RIGHT = 2

class AimingMotor(HBridge):
    """
    A class to control the tilting mechanism using an H-Bridge motor driver.

    This class extends HBridge to control a motor that adjusts tilt.

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
        Initializes the linear actuator control system by setting up the H-Bridge motor.

        Args:
            in1 (int): GPIO pin for motor direction.
            in2 (int): GPIO pin for motor direction.
            enable (int): GPIO pin for enabling the motor.
            pwm_freq (int): Frequency of the PWM signal in Hz (default: constants.LINEAR_ACTUATOR_FREQ).
            pwm_dc (int): Initial duty cycle (0-100%) (default: constants.LINEAR_ACTUATOR_DC).
        """
        super().__init__(in1, in2, enable, pwm_freq, min_duty_cycle, max_duty_cycle, pi, pwm_range)

        self.left_limit_switch = left_limit_switch
        self.right_limit_switch = right_limit_switch
        self.debouncing = threading.Event()

        self.pi.set_mode(self.left_limit_switch, pigpio.INPUT)
        self.pi.set_mode(self.right_limit_switch, pigpio.INPUT)

        self.pi.set_pull_up_down(self.left_limit_switch, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.right_limit_switch, pigpio.PUD_UP)

        self.pi.callback(self.left_limit_switch, pigpio.EITHER_EDGE, self.limit_switch_ISR)
        self.pi.callback(self.right_limit_switch, pigpio.EITHER_EDGE, self.limit_switch_ISR)

        limit_switch_thread = threading.Thread(target=self.limit_switch_state_machine)
        limit_switch_thread.daemon = True
        limit_switch_thread.start()

        if not self.pi.read(self.left_limit_switch):
            self.status = TOO_FAR_LEFT
            self.right(RESET_LIMIT_SWITCH_SPEED)
        elif not self.pi.read(self.right_limit_switch):
            self.status = TOO_FAR_RIGHT
            self.left(RESET_LIMIT_SWITCH_SPEED)
        else:
            self.status = OK

    def limit_switch_state_machine(self):
        while True:
            if self.debouncing.wait(): 
                if self.status == TOO_FAR_LEFT:
                    self.right(RESET_LIMIT_SWITCH_SPEED)
                elif self.status == TOO_FAR_RIGHT:
                    self.left(RESET_LIMIT_SWITCH_SPEED)
                else:
                    self.stop()

                time.sleep(0.1)
                self.debouncing.clear()            

    def limit_switch_ISR(self, GPIO, level, _tick):
        if not self.debouncing.is_set():
            if level == 0:
                if GPIO == self.left_limit_switch:
                    self.status = TOO_FAR_LEFT
                else:
                    self.status = TOO_FAR_RIGHT
            else:
                self.status = OK

            self.debouncing.set()

    def right(self, speed):
        if self.status != TOO_FAR_RIGHT:
            self.forward(speed)

    def left(self, speed):
        if self.status != TOO_FAR_LEFT:
            self.backward(speed)
