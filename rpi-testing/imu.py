import time
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x.uart import BNO08X_UART
from scipy.spatial.transform import Rotation as R


def initialize_imu():
    """
    Initialize IMU to get readings.

    :return: IMU initialization
    """
    print("initializing...")
    try:
        # Initialize I2C
        i2c = busio.I2C(board.SCL, board.SDA)
        print("i2c initialized")
        bno = BNO08X_I2C(i2c)
        print("bno initialized")

        # Enable Quaternion readings for sensor
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        print("enabled")

        return bno

    except Exception as e:
        print(f'IMU initialization failed: {e}')
    return None

def read_quaternion(bno):
    """
    Get Quaternion readings from IMU.

    :param bno: Initialized sensor
    :return: Quaternion readings
    """

    try:
        quat_i, quat_j, quat_k, quat_real = bno.quaternion

        return quat_i, quat_j, quat_k, quat_real
    
    except KeyError as e:
        print(f'KeyError: IMU returned unexpected data format: {e}')
    except OSError as e:
        print(f'OSError: Possible I2C disconnection: {e}')
    except Exception as e:
        print(f'Unexpected error reading gyro: {e}')

    return None


def quaternion_to_euler(w, x, y, z):
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


if __name__ == '__main__':

    bno = initialize_imu()

    # Output readings and angles
    while True:
        print("Rotation Vector Quaternion:")
        try:
            quat_i, quat_j, quat_k, quat_real = read_quaternion(bno)
            print(f'I: {quat_i:0.6f} J: {quat_j:0.6f} K: {quat_k:0.6f} Real: {quat_real:0.6f}')

            if all((quat_i, quat_j, quat_k, quat_real)):
                yaw, pitch, roll = quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
                print(f'Yaw: {yaw:0.6f} Pitch: {pitch:0.6f} Roll: {roll:0.6f}')

                print("")

        except Exception as e:
            print(f"exception occurred: {e}")

        time.sleep(0.1)
