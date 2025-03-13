from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x_rvc import BNO08x_RVC
import board
import busio
import serial
from collections import deque
from scipy.spatial.transform import Rotation as R
import pigpio

import constants
import time
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

    def __init__(self, reset_pin, uart=None, pi=None, buffer_size=constants.IMU_BUFFER_SIZE): 
        """
        Initializes the buffers for smoothing and IMU sensor for readings.

        Args:
            buffer_size (int): the maximum number of past readings to store for smoothing.
        """
        # self.yaw_buffer = deque(maxlen=buffer_size)
        # self.pitch_buffer = deque(maxlen=buffer_size)
        # self.roll_buffer = deque(maxlen=buffer_size)
        self.reset_pin = reset_pin
        self.yaw = None
        self.pitch = None
        self.roll = None

        if not pi:
            self.pi = pigpio.pi()
        else:
            self.pi = pi
    
        self.uart, self.rvc = self.initialize_imu(uart)
        measure_thread = threading.Thread(target=self.measure_thread)
        measure_thread.daemon = True
        measure_thread.start()


    def measure_thread(self):
        while True:
            self.roll, self.pitch, self.yaw, _, __, ___ = self.rvc.heading
            time.sleep(0.02)


    def initialize_imu(self, uart):
        """
        Initializes the I2C and the BNO085 sensor to get readings.

        This method sets up the I2C connection and configures the BNO085 sensor 
        to enable quaternion readings for tracking orientation.
        
        Returns:
            Tuple[busio.I2C, BNO08X_I2C]: 
            A tuple containing the initialized I2C bus and BNO08X sensor instance.
            Returns (None, None) if initialization fails.
        
        Raises:
            Exception: If the I2C bus or BNO085 sensor fails to initialize.
        """
        try:
            # Initialize I2C
            print("starting initialization")
            if not uart:
                uart = serial.Serial("/dev/serial0", 115200)
                print("initializing uart")
            print("uart initialized")

            self.pi.set_mode(self.reset_pin, pigpio.OUTPUT)

            rvc = BNO08x_RVC(uart)
            print("RVC initialized")

            # Enable Quaternion readings for sensor
            yaw, pitch, roll, x_accel, y_accel, z_accel = rvc.heading
            print("Yaw: %2.2f Pitch: %2.2f Roll: %2.2f Degrees" % (yaw, pitch, roll))
            print("Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel))


            return uart, rvc

        except Exception as e:
            print(f'IMU initialization failed: {e}')
            return None, None

    def reset(self):
        self.pi.write(self.reset_pin, 1)
        time.sleep(0.1)
        self.pi.write(self.reset_pin, 0)
        time.sleep(0.1)
        self.pi.write(self.reset_pin, 1)
        time.sleep(0.1)

    def imu_readings(self):
        """
        Get IMU readings (yaw, pitch, roll).

        Returns:
            Tuple[Optional[float], Optional[float], Optional[float]]: 
            Euler angles (yaw, pitch, roll), or (None, None, None) if an error occurs.
        """

        # Get Quaternion readings from IMU
        # yaw, pitch, roll, x_accel, y_accel, z_accel = self.rvc.heading
        # print("Yaw: %2.2f Pitch: %2.2f Roll: %2.2f Degrees" % (yaw, pitch, roll))
        # print("Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel))
        # print(f'I: {quat_i:0.6f} J: {quat_j:0.6f} K: {quat_k:0.6f} Real: {quat_real:0.6f}')

        # if None in (quat_i, quat_j, quat_k, quat_real):
        #     print("IMU quaternion reading failed. Returning None values for yaw, pitch, and roll.")
        #     return None, None, None

        # Convert Quaternion readings to Euler angles
        # for i in range(5):
        try:
            return self.roll, self.pitch, self.yaw
        except Exception as e:
            print(f"Exception {e}")
            return None, None, None


    def smooth_readings(self):
        """
        Smooth IMU readings using a rolling average filter.

        Returns:
            Tuple[Optional[float], Optional[float], Optional[float]]: 
            Smoothed (yaw, pitch, roll) values or (None, None, None) if no valid data.
        
        Raises:
            ZeroDivisionError: If the buffer is empty
            TypeError: If unexpected data types exist
        """
        try:
            yaw, pitch, roll = self.imu_readings()

            # Ensure valid values before appending to buffer
            if None not in (yaw, pitch, roll):
                self.yaw_buffer.append(yaw)
                self.pitch_buffer.append(pitch)
                self.roll_buffer.append(roll)
            else:
                print("Warning: IMU readings returned None, skipping buffer update.")

            averaged_yaw = sum(self.yaw_buffer) / len(self.yaw_buffer)
            averaged_pitch = sum(self.pitch_buffer) / len(self.pitch_buffer)
            averaged_roll = sum(self.roll_buffer) / len(self.roll_buffer)
            print(f'Yaw: {averaged_yaw:.6f} Pitch: {averaged_pitch:.6f} Roll: {averaged_roll:.6f}')

            return averaged_yaw, averaged_pitch, averaged_roll

        except ZeroDivisionError as e:
            print(f'Error: No IMU values recorded yet: {e}')
        except TypeError as e:
            print(f'Error: Issue with buffer data type: {e}')
        return None, None, None

if __name__ == '__main__':
    import time
    bno = IMU()

    # Output readings and angles
    while True:
        print("Rotation Vector Quaternion:")
        try:
            # quat_i, quat_j, quat_k, quat_real = bno.read_quaternion()
            # print(f'I: {quat_i:0.6f} J: {quat_j:0.6f} K: {quat_k:0.6f} Real: {quat_real:0.6f}')

            # if all((quat_i, quat_j, quat_k, quat_real)):
            #     yaw, pitch, roll = bno.quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
            #     print(f'Yaw: {yaw:0.6f} Pitch: {pitch:0.6f} Roll: {roll:0.6f}')

            #     print("")
            yaw, pitch, roll, x_accel, y_accel, z_accel = bno.rvc.heading
            print("Yaw: %2.2f Pitch: %2.2f Roll: %2.2f Degrees" % (yaw, pitch, roll))
            print("Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel))

        except Exception as e:
            print(f"exception occurred: {e}")

        time.sleep(0.1)