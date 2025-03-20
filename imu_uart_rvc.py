from adafruit_bno08x_rvc import BNO08x_RVC
import serial
import pigpio

import time
from collections import deque
import threading

class IMU:
    """
    A class to obtain readings from BNO085 IMU sensor.

    Attributes:
        yaw_buffer (deque): list to store yaw values
        pitch_buffer (deque): list to store pitch values
        roll_buffer (deque): list to store roll values
        i2c (busio.I2C): instance of I2C class to obtain sensor readings
        bno (BNO08X_I2C): BNO085 sensor
    """

    def __init__(self, reset_pin, uart=None, pi=None): 
        """
        Initializes the buffers for smoothing and IMU sensor for readings.

        Args:
            buffer_size (int): the maximum number of past readings to store for smoothing.
        """
        if not pi:
            self.pi = pigpio.pi()
        else:
            self.pi = pi

        self.reset_pin = reset_pin
        self.pi.set_mode(reset_pin, pigpio.OUTPUT)
        self.reset()

        self.yaw_deque = deque(maxlen=50) # each entry is 0.01s apart, covers 0.1s history
        self.pitch = None
        self.roll = None

        self.new_data = threading.Event()

        try:
            # Initialize I2C
            print("Starting IMU RVC initialization")
            if not uart:
                print("Initializing UART")
                uart = serial.Serial("/dev/serial0", 115200)
                print("uart initialized")

            self.rvc = BNO08x_RVC(uart)
            print("RVC initialized")

            # Enable Quaternion readings for sensor
            measure_thread = threading.Thread(target=self.measure_thread)
            measure_thread.daemon = True
            measure_thread.start()
            if not self.new_data.wait(1):
                raise Exception("No new data received")
        
            print("IMU initialized")

        except Exception as e:
            print(f'IMU initialization failed: {e}')

    def measure_thread(self):
        while True:
            yaw, self.roll, self.pitch, _, __, ___ = self.rvc.heading
            self.yaw_deque.appendleft(yaw)
            self.new_data.set()
            self.new_data.clear()
            time.sleep(0.005) # new data every 10ms

    def reset(self):
        print("Resetting IMU...")
        self.pi.write(self.reset_pin, 1)
        time.sleep(0.1)
        self.pi.write(self.reset_pin, 0)
        time.sleep(0.1)
        self.pi.write(self.reset_pin, 1)
        time.sleep(0.1)
        print("IMU reset")

    def imu_readings(self):
        """
        Get IMU readings (yaw, pitch, roll).

        Returns:
            Tuple[Optional[float], Optional[float], Optional[float]]: 
            Euler angles (yaw, pitch, roll), or (None, None, None) if an error occurs.
        """
        return self.yaw, self.roll, self.pitch
    
    @property
    def yaw(self):
        return self.yaw_deque[0]

if __name__ == '__main__':
    rvc = IMU(reset_pin=4)

    # Output readings and angles
    start_time = time.time()
    while True:
        rvc.new_data.wait()
        print(f"{time.time() - start_time}: {rvc.imu_readings()}")