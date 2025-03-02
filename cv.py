import cv2
import numpy as np
import os
import mediapipe as mp
import time
import RPi.GPIO as GPIO
import constants
from collections import deque


class CV():
    def __init__(self):
        # Initialize Mediapipe
        try:
            self.mp_pose = mp.solutions.pose
            self.pose = self.mp_pose.Pose()

        except Exception as e:
            print(f"Error initializing Mediapipe Pose: {e}")
            self.pose = None

        # OpenCV
        # Initialize camera
        self.cap = cv2.VideoCapture(0)  # Use 0 for the default camera
        
        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            self.cap = None  # Prevent further errors if camera fails
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.distance_buffer = deque(maxlen=constants.BUFFER_SIZE)

    def read_frame(self):
        if self.cap is None:
            print("Camera not initialized.")
            return None
        
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            return None

        return frame


    def process_frame(self, frame):
        if self.pose is None:
            print("Pose model not initialized.")
            return None

        try:
            # Convert the image from BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # Get pose model results
            pose_results = self.pose.process(frame_rgb)

            return frame_rgb, pose_results

        except Exception as e:
            print(f"Error processing frame: {e}")
            return None, None

    def extract_joints(self, pose_results):

        if pose_results.pose_landmarks:

            joints = {
                'body': (
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER],
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER],
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_HIP],
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_HIP]
                ),
                'left arm': (
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_WRIST],
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_ELBOW]
                ),
                'right arm': (
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST],
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_ELBOW]
                ),
                'nose': pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.NOSE]
            }

            return joints

        return None

    def calculate_length(self, joint_1, joint_2):
        """
        Calculate the distance between 2 joints

        :param joint_1: joint 1
        :param joint_2: joint 2
        :return: distance as a normalized value based on the camera feed
        """
        joint_1 = np.array([joint_1.x, joint_1.y, joint_1.z])
        joint_2 = np.array([joint_2.x, joint_2.y, joint_2.z])

        joint_2_to_1 = joint_1 - joint_2

        length = np.linalg.norm(joint_2_to_1)

        return length

    def estimate_distance(self, frame, joints):

        # if joints['right arm'][0] and joints['right arm'][1]:
        try:
            length = self.calculate_length(joints['right arm'][0], joints['right arm'][1])
            distance_to_object = (4 * 280 * 720) / (length * 480 * 2.02) / 1000
            
            cv2.putText(frame, f'Estimated Distance: {distance_to_object:.2f}m', (0, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            return distance_to_object

        except TypeError as e:
            print(f'TypeError: Joint Not Found {e}')
        return None

    def smooth_distance(self):
        pass

    def pose_estimation(self, frame, joints):
        if joints is None:# or 'body' not in joints or 'nose' not in joints:
            return None

        # Check for centering
        if all(element.visibility > 0.1 for element in joints['body']):
            body_position = sum(element.x for element in joints['body']) / len(joints['body'])
            
            if constants.CENTER - constants.POSE_TOLERANCE < body_position < constants.CENTER + constants.POSE_TOLERANCE:
                if any(element.y < joints['nose'].y for element in joints['right arm']) or \
                   any(element.y < joints['nose'].y for element in joints['left arm']):

                    cv2.putText(frame, 'Pose Estimation: centered and throw identified', (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    return 'centered and throw identified'
                else:
                    cv2.putText(frame, 'Pose Estimation: centered', (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    return 'centered'
            else:
                direction = 'move left' if body_position < constants.CENTER else 'move right'
                cv2.putText(frame, f'Pose Estimation: {direction}', (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                return direction
        else:
            cv2.putText(frame, f'Pose Estimation: torso not found', (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            return None

    def cap_release(self):
        self.cap.release()


if __name__ == "__main__":

    cv = CV()

    while True:

        frame = cv.read_frame()
        frame_rgb, pose_results = cv.process_frame(frame)
        joints = cv.extract_joints(pose_results)

        distance = cv.estimate_distance(frame, joints)
        print(f'Distance: {distance}m')
        
        pose_estimation = cv.pose_estimation(frame, joints)
        print(f'Pose Estimation: {pose_estimation}')

        # Show the video feed with the landmarks
        cv2.imshow("Frisbeast Vision", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv.cap_release()
    cv2.destroyAllWindows()
