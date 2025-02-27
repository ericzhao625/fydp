from inputs import initialize_imu, read_quaternion, imu_readings
import time
from collections import deque

refresh_rate = 0.5
moving_average_window = 10
yaw_buffer = deque(maxlen=moving_average_window)
pitch_buffer = deque(maxlen=moving_average_window)
roll_buffer = deque(maxlen=moving_average_window)

if __name__ == '__main__':

    bno = initialize_imu()

    while True:
        try:
            yaw, pitch, roll = imu_readings(bno)

            # Smoothen values
            yaw_buffer.append(yaw)
            pitch_buffer.append(pitch)
            roll_buffer.append(roll)

            averaged_yaw = sum(yaw_buffer) / len(yaw_buffer)
            averaged_pitch = sum(pitch_buffer) / len(pitch_buffer)
            averaged_roll = sum(roll_buffer) / len(roll_buffer)

            print(f'Yaw: {averaged_yaw:.6f} Pitch: {averaged_pitch:.6f} Roll: {averaged_roll:.6f}')
        except Exception as e:
            print(f'Exception: IMU did not get data readings: {e}')

        time.sleep(refresh_rate)
