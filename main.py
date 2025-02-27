import time
from collections import deque
import RPi.GPIO as GPIO

from imu import initialize_imu, smooth_readings
from throwing import throwing_speed

# IMU
refresh_rate = 0.5
moving_average_window = 10
yaw_buffer = deque(maxlen=moving_average_window)
pitch_buffer = deque(maxlen=moving_average_window)
roll_buffer = deque(maxlen=moving_average_window)

# Throwing
PWM_PIN = 13  # Physical pin 33
PWM_FREQ = 500
PWM_INIT_DC = 0


if __name__ == '__main__':

    bno = initialize_imu()

    # Setup
    GPIO.setmode(GPIO.BCM)  # Use Broadcom GPIO pin numbering
    GPIO.setup(PWM_PIN, GPIO.OUT)

    # Initialize PWM on the pin with a frequency of 100 Hz
    pwm = GPIO.PWM(PWM_PIN, 500)
    pwm.start(PWM_INIT_DC)  # Start PWM with 0% duty cycle

    while True:
        yaw, pitch, roll = smooth_readings(bno, yaw_buffer, pitch_buffer, roll_buffer)

        distance = 10   # To be replaced with CV distance estimate
        
        pwm.ChangeDutyCycle(throwing_speed(distance))
        
        time.sleep(refresh_rate)
