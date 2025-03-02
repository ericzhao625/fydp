import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C
from scipy.spatial.transform import Rotation as R
from collections import deque


class IMU:
    def __init__(self, buffer_size=10):
        """
        Initialize the IMU sensor and buffers for smoothing.
        """
        self.buffer_size = buffer_size
        self.yaw_buffer = deque(maxlen=buffer_size)
        self.pitch_buffer = deque(maxlen=buffer_size)
        self.roll_buffer = deque(maxlen=buffer_size)
    
        self.bno = self.initialize_imu()

    def initialize_imu(self):
        """
        Initialize IMU to get readings.

        :return: IMU initialization
        """
        try:
            # Initialize I2C
            i2c = busio.I2C(board.SCL, board.SDA)
            bno = BNO08X_I2C(i2c)

            # Enable Quaternion readings for sensor
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

            return bno

        except Exception as e:
            print(f'IMU initialization failed: {e}')
            return None


    def read_quaternion(self):
        """
        Get Quaternion readings from IMU.

        :param bno: Initialized sensor
        :return: Quaternion readings
        """

        try:
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion

            return quat_i, quat_j, quat_k, quat_real
        
        except KeyError as e:
            print(f'KeyError: IMU returned unexpected data format: {e}')
        except OSError as e:
            print(f'OSError: Possible I2C disconnection: {e}')
        except Exception as e:
            print(f'Unexpected error reading gyro: {e}')

        return None


    def quaternion_to_euler(self, w, x, y, z):
        """
        Convert Quaternion readings to Euler angles.

        :param w: scalar (real) part
        :param x: vector (imaginary) part
        :param y: vector (imaginary) part
        :param z: vector (imaginary) part
        :return: yaw, pitch, roll
        """
        
        quaternion_readings = R.from_quat([x, y, z, w])
        euler_angles = quaternion_readings.as_euler('zyx', degrees=True)

        return euler_angles


    def imu_readings(self):
        """
        Get IMU readings.

        :param bno: Initialized sensor
        :return: yaw, pitch, roll
        """

        # Get Quaternion readings from IMU
        quat_i, quat_j, quat_k, quat_real = self.read_quaternion()
        # print(f'I: {quat_i:0.6f} J: {quat_j:0.6f} K: {quat_k:0.6f} Real: {quat_real:0.6f}')

        # Convert Quaternion readings to Euler angles
        yaw, pitch, roll = self.quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
        # print(f'Yaw: {yaw:0.6f} Pitch: {pitch:0.6f} Roll: {roll:0.6f}')

        return yaw, pitch, roll


    def smooth_readings(self):
        """
        Remove outliers by using a low pass filter.

        :return: averaged yaw, pitch, and roll values
        """
        try:
            yaw, pitch, roll = self.imu_readings()

            self.yaw_buffer.append(yaw)
            self.pitch_buffer.append(pitch)
            self.roll_buffer.append(roll)

            averaged_yaw = sum(self.yaw_buffer) / len(self.yaw_buffer)
            averaged_pitch = sum(self.pitch_buffer) / len(self.pitch_buffer)
            averaged_roll = sum(self.roll_buffer) / len(self.roll_buffer)
            print(f'Yaw: {averaged_yaw:.6f} Pitch: {averaged_pitch:.6f} Roll: {averaged_roll:.6f}')

            return averaged_yaw, averaged_pitch, averaged_roll

        except ValueError as e:
            print(f'Exception: IMU did not get data readings: {e}')
        except TypeError as e:
            print(f'Exception: {e}')
        return None, None, None
