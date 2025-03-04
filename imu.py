from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C
import board
import busio
from collections import deque
from scipy.spatial.transform import Rotation as R

import constants


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

    def __init__(self, buffer_size=constants.IMU_BUFFER_SIZE): 
        """
        Initializes the buffers for smoothing and IMU sensor for readings.

        Args:
            buffer_size (int): the maximum number of past readings to store for smoothing.
        """
        self.yaw_buffer = deque(maxlen=buffer_size)
        self.pitch_buffer = deque(maxlen=buffer_size)
        self.roll_buffer = deque(maxlen=buffer_size)
    
        self.i2c, self.bno = self.initialize_imu()

    def initialize_imu(self):
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
            i2c = busio.I2C(board.SCL, board.SDA)
            bno = BNO08X_I2C(i2c)

            # Enable Quaternion readings for sensor
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

            return i2c, bno

        except Exception as e:
            print(f'IMU initialization failed: {e}')
            return None, None


    def read_quaternion(self):
        """
        Get Quaternion readings from IMU.

        Returns:
            Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]: 
            Quaternion components (i, j, k, real), or (None, None, None, None) if an error occurs.

        Raises:
            KeyError: If the IMU returns an unexpected data format.
            OSError: If there's a possible I2C disconnection.
            Exception: For any other unexpected errors.
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

        return None, None, None, None


    def quaternion_to_euler(self, w, x, y, z):
        """
        Converts a quaternion (w, x, y, z) into Euler angles (yaw, pitch, roll).

        The Euler angles are computed using the 'ZYX' intrinsic rotation sequence, where:
            - Yaw is the rotation around the Z-axis.
            - Pitch is the rotation around the Y-axis.
            - Roll is the rotation around the X-axis.

        Args:
            w (float): Scalar (real) part of the quaternion.
            x (float): X component (imaginary part).
            y (float): Y component (imaginary part).
            z (float): Z component (imaginary part).

        Returns:
            Tuple[float, float, float]: Euler angles (yaw, pitch, roll) in degrees.
        """
        
        quaternion_readings = R.from_quat([x, y, z, w])
        euler_angles = quaternion_readings.as_euler('zyx', degrees=True)

        return euler_angles


    def imu_readings(self):
        """
        Get IMU readings (yaw, pitch, roll).

        Returns:
            Tuple[Optional[float], Optional[float], Optional[float]]: 
            Euler angles (yaw, pitch, roll), or (None, None, None) if an error occurs.
        """

        # Get Quaternion readings from IMU
        quat_i, quat_j, quat_k, quat_real = self.read_quaternion()
        # print(f'I: {quat_i:0.6f} J: {quat_j:0.6f} K: {quat_k:0.6f} Real: {quat_real:0.6f}')

        if None in (quat_i, quat_j, quat_k, quat_real):
            print("IMU quaternion reading failed. Returning None values for yaw, pitch, and roll.")
            return None, None, None

        # Convert Quaternion readings to Euler angles
        yaw, pitch, roll = self.quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
        # print(f'Yaw: {yaw:0.6f} Pitch: {pitch:0.6f} Roll: {roll:0.6f}')

        return yaw, pitch, roll


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

    def cleanup(self):
        """
        Clean up IMU.

        Raises:
            AttributeError: If there is an I2C error
            Exception: If there is an unknown error
        """

        print('Stopping IMU')

        try:
            if self.i2c:
                self.i2c.unlock()  # Unlock I2C bus if it's locked
                print("I2C bus unlocked.")

            self.i2c = None  # Set reference to None

        except AttributeError:
            print("Warning: I2C object does not support cleanup or was not initialized.")
        except Exception as e:
            print(f"Error during IMU cleanup: {e}")

        print('IMU stopped')
