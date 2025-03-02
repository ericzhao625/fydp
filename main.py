import time
from collections import deque
import RPi.GPIO as GPIO

import cv2
import mediapipe as mp

from imu import IMU
from throwing import throwing_speed
from cv import CV

import constants


if __name__ == '__main__':

    # Initialize IMU
    imu = IMU()

    # Initialize CV
    cv = CV()

    while True:
        frame = cv.read_frame()
        frame_rgb, pose_results = cv.process_frame(frame)
        joints = cv.extract_joints(pose_results)

        distance = cv.smooth_distance(frame, joints)
        # print(f'Distance: {distance}m')

        pose_estimation = cv.pose_estimation(frame, joints)
        # print(f'Pose Estimation: {pose_estimation}')

        yaw, pitch, roll = imu.smooth_readings()

        # Show the video feed with the landmarks
        cv2.imshow("Frisbeast Vision", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv.cap_release()
    cv2.destroyAllWindows()
