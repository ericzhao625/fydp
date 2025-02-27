from inputs import initialize_imu, smooth_readings
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
        yaw, pitch, roll = smooth_readings(bno, yaw_buffer, pitch_buffer, roll_buffer)

        time.sleep(refresh_rate)
