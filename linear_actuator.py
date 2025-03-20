from h_bridge import HBridge, FORWARD, REVERSE

from adafruit_ina3221 import INA3221
import threading
import time

FREQ = 10000
MIN_DUTY_CYCLE = 100
MAX_DUTY_CYCLE = 100

BOTTOM = 0
MIDDLE = 1
TOP = 2
UNKNOWN = 3

UP_TIME = 14 # time to move from bottom to top
DOWN_TIME = 11

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
        ina: INA3221=None,
        ina_channel: int = None,
        i2c_lock: threading.Lock=None,
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
        """
        super().__init__(in1, in2, enable, pwm_freq, min_duty_cycle, max_duty_cycle, pi, pwm_range)
        
        self.state = UNKNOWN
        self.i2c_lock = i2c_lock
        self.in_movement = False

        if ina is not None:
            self.ina = ina
            self.ina_channel = ina_channel
            state_monitor_thread = threading.Thread(target=self.state_monitor)
            state_monitor_thread.daemon = True
            state_monitor_thread.start()
        else:
            self.ina = None

    def calibrate(self, wait=True):
        return self.move_to_bottom(wait)

    def delay_thread(self, wait_time, end_state):
        if self.ina is not None:
            self.stopped.wait()
        elif self.state == MIDDLE:
            time.sleep(wait_time / 2 + 0.5)
        else:
            time.sleep(wait_time + 0.5)
        
        self.state = end_state

    def move_to_bottom_thread(self):
        if self.state != BOTTOM:
            self.down(100)
            if self.ina is not None:
                self.stopped.wait()
            elif self.state == MIDDLE:
                time.sleep(DOWN_TIME / 2 + 0.5)
            else:
                time.sleep(DOWN_TIME + 0.5)

        self.state = BOTTOM
        self.in_movement = False

    def move_to_bottom(self, wait=True):
        if self.in_movement:
            return
        else:
            self.in_movement = True

        delay_thread = threading.Thread(target=self.move_to_bottom_thread)
        delay_thread.start()
        if wait:
            delay_thread.join()
        else:
            return delay_thread

    def move_to_top_thread(self):
        if self.state != TOP:
            self.up(100)
            if self.ina is not None:
                self.stopped.wait()
            elif self.state == MIDDLE:
                time.sleep(UP_TIME / 2 + 0.5)
            else:
                time.sleep(UP_TIME + 0.5)

        self.state = TOP
        self.in_movement = False

    def move_to_top(self, wait=True):
        if self.in_movement:
            return
        else:
            self.in_movement = True

        delay_thread = threading.Thread(target=self.move_to_top_thread)
        delay_thread.start()
        if wait:
            delay_thread.join()
        else:
            return delay_thread

    def move_to_middle_thread(self):
        if self.state == UNKNOWN:
            self.in_movement = False
            self.calibrate()
            self.in_movement = True

        if self.state == BOTTOM:
            self.up(100)
            time.sleep(UP_TIME / 2)
        elif self.state == TOP:
            self.down(100)
            time.sleep(DOWN_TIME / 2)
        
        self.stop()
        self.state = MIDDLE
        self.in_movement = False

    def move_to_middle(self, wait=True):
        if self.in_movement:
            return
        else:
            self.in_movement = True

        delay_thread = threading.Thread(target=self.move_to_middle_thread)
        delay_thread.start()
        if wait:
            delay_thread.join()
        else:
            return delay_thread

    def state_monitor(self):
        while True:
            self.i2c_lock.acquire()
            try:
                self.ina.mode = 1
                time.sleep(0.02)

                if abs(self.ina[self.ina_channel].current_amps) < 0.010:
                    if self.direction == FORWARD:
                        self.state = TOP
                        # self.stop()
                    elif self.direction == REVERSE:
                        self.state = BOTTOM
                        # self.stop()

                    self.moving.clear()
                    self.stopped.set()
                
                self.i2c_lock.release()
            except Exception as e:
                print(f"Exception occurred in linear actuator thread: {e}")
                self.i2c_lock.release()

            self.moving.wait()
            # Don't need to poll that often, but can shorten or remove if needed
            time.sleep(0.1)

    def up(self, speed):
        if self.state != TOP:
            self.state = UNKNOWN
        self.forward(speed)

    def down(self, speed):
        if self.state != BOTTOM:
            self.state = UNKNOWN

        self.reverse(speed)
