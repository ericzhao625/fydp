from h_bridge import HBridge, FORWARD, REVERSE

from adafruit_ina3221 import INA3221
import threading
import time

FREQ = 10000
MIN_DUTY_CYCLE = 80
MAX_DUTY_CYCLE = 100

BOTTOM = 0
MIDDLE = 1
TOP = 2
UNKNOWN = 3

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
        self.moving = threading.Event()
        self.i2c_lock = i2c_lock

        if ina is not None:
            self.ina = ina
            self.ina_channel = ina_channel
            state_monitor_thread = threading.Thread(target=self.state_monitor)
            state_monitor_thread.daemon = True
            state_monitor_thread.start()

    def state_monitor(self):
        while True:
            self.i2c_lock.acquire()
            try:
                self.ina.mode = 1
                time.sleep(0.02)

                if abs(self.ina[self.ina_channel].current_amps) < 0.010:
                    if self.direction == FORWARD:
                        self.state = TOP
                        self.stop()
                    elif self.direction == REVERSE:
                        self.state = BOTTOM
                        self.stop()

                    self.moving.clear()
                
                self.i2c_lock.release()
            except Exception as e:
                print(f"Exception occurred in linear actuator thread: {e}")
                self.i2c_lock.release()

            self.moving.wait()
            # Don't need to poll that often, but can shorten or remove if needed
            time.sleep(0.1)

    def up(self, speed):
        self.forward(speed)
        self.moving.set()

    def down(self, speed):
        self.reverse(speed)
        self.moving.set()
