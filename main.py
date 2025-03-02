import time
from collections import deque
import RPi.GPIO as GPIO

import cv2
import mediapipe as mp

from imu import initialize_imu, smooth_readings
from throwing import throwing_speed
from cv import pose_detection

import constants

# IMU
refresh_rate = 0.5
moving_average_window = 10
yaw_buffer = deque(maxlen=moving_average_window)
pitch_buffer = deque(maxlen=moving_average_window)
roll_buffer = deque(maxlen=moving_average_window)

if __name__ == '__main__':

    pinLED = 17
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(pinLED, GPIO.OUT)

    # # MediaPipe
    # center = 0.5
    # pose_tolerance = 0.2

    # Initialize MediaPipe Pose
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose()

    # Initialize camera
    cap = cv2.VideoCapture(0)  # Use 0 for the default camera

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cap.set(cv2.CAP_PROP_FPS, 5)  # Attempt to set FPS to 30
    fps = cap.get(cv2.CAP_PROP_FPS)  # Check if the setting was successful
    print(f"Camera FPS: {fps}")

    prev_time = 0

    while cap.isOpened():
        ret, frame = cap.read()

        # frame = cv2.resize(frame, (640, 480))
        # Flip the frame horizontally
        # flipped_frame = cv2.flip(frame, 1)
        if not ret:
            break

        distance = pose_detection(mp_pose, pose, frame)

        print(distance)
        # Display memory usage
        # cv2.putText(frame, monitor_memory(), (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show the video feed with the landmarks
        cv2.imshow("Pose Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
