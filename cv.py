from collections import deque
import cv2
import mediapipe as mp
import numpy as np
import os
import RPi.GPIO as GPIO
import time

import constants


class CV():
    """
    A class for real-time pose estimation and distance estimation using OpenCV and Mediapipe.

    This class handles video capture, pose detection, joint extraction, distance estimation, 
    and movement direction prediction.
    """
    def __init__(self):
        """
        Initializes the CV class by setting up Mediapipe Pose model and OpenCV camera capture.
        """
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

        self.distance_buffer = deque(maxlen=constants.CV_BUFFER_SIZE)

    def read_frame(self):
        """
        Captures a frame from the camera.

        Returns:
            frame (np.ndarray or None): Captured frame, or None if the capture fails.
        """
        if self.cap is None:
            print("Camera not initialized.")
            return None
        
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            return None

        return frame


    def process_frame(self, frame):
        """
        Processes a frame to detect human pose landmarks.

        Args:
            frame (np.ndarray): Input image frame in BGR format.

        Returns:
            Tuple[np.ndarray, mediapipe.framework.formats.landmark_pb2.NormalizedLandmarkList or None]:
            - frame_rgb: Converted RGB image.
            - pose_results: Pose landmarks detected by Mediapipe, or None if detection fails.
        """
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
        """
        Extracts key joints from the detected pose landmarks.

        Args:
            pose_results (mediapipe.framework.formats.landmark_pb2.NormalizedLandmarkList): 
            The result from Mediapipe pose detection.

        Returns:
            dict or None: A dictionary containing extracted joint positions or None if no landmarks are detected.
        """
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
                'nose': pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.NOSE],
                'left leg': (
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_HIP],
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_KNEE],
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_ANKLE]
                ),
                'right leg': (
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_HIP],
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_KNEE],
                    pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_ANKLE]
                )
            }

            return joints

        return None

    def calculate_length(self, joint_1, joint_2):
        """
        Computes the Euclidean distance between two joints.

        Args:
            joint_1 (mediapipe.framework.formats.landmark_pb2.NormalizedLandmark): First joint.
            joint_2 (mediapipe.framework.formats.landmark_pb2.NormalizedLandmark): Second joint.

        Returns:
            float: Distance between the two joints.
        """
        joint_1 = np.array([joint_1.x, joint_1.y, joint_1.z])
        joint_2 = np.array([joint_2.x, joint_2.y, joint_2.z])

        joint_2_to_1 = joint_1 - joint_2

        length = np.linalg.norm(joint_2_to_1)

        return length

    def estimate_distance(self, frame, joints):
        """
        Estimates the distance of the player from the camera based on arm length.

        Args:
            frame (np.ndarray): Captured frame.
            joints (dict): Dictionary containing detected joint positions.

        Returns:
            float or None: Estimated distance or None if joints are missing.
        
        Raises:
            TypeError: If joints is None (joints are missing)
        """
        try:
            average_torso_length = (
                self.calculate_length(joints['body'][0], joints['body'][2]) +
                self.calculate_length(joints['body'][0], joints['body'][2])
            )/2
            average_femur_length = (
                self.calculate_length(joints['left leg'][0], joints['left leg'][1]) +
                self.calculate_length(joints['right leg'][0], joints['right leg'][1])
            )/2
            average_tibia_length = (
                self.calculate_length(joints['left leg'][1], joints['left leg'][2]) +
                self.calculate_length(joints['right leg'][1], joints['right leg'][2])
            )/2
            height = average_torso_length + average_femur_length + average_ibia_length
            print(f'Height: {height}')

            distance_to_object = (constants.CAMERA_FOCAL_LENGTH * constants.USER_HEIGHT * constants.HEIGHT_TO_SHOULDERS * constants.CAMERA_PIXEL_HEIGHT) / ((length * constants.CAMERA_PIXEL_HEIGHT) * constants.CAMERA_SENSOR_HEIGHT) / 1000            

            return distance_to_object

        except TypeError as e:
            print(f'TypeError: Joints Not Found {e}')
        return None

    def smooth_distance(self, frame, joints):
        """
        Smooths the estimated distance using a rolling average filter.

        Args:
            frame (np.ndarray): Captured frame.
            joints (dict): Dictionary containing detected joint positions.

        Returns:
            float or None: Smoothed distance value.

        Raises:
            ZeroDivisionError: If distance buffer is empty
        """
        distance = self.estimate_distance(frame, joints)

        if distance is not None:
            self.distance_buffer.append(distance)
        try:
            averaged_distance = sum(self.distance_buffer) / len(self.distance_buffer)
            return averaged_distance

        except ZeroDivisionError as e:
            print('No distance values recorded yet.')
        return None

    def pose_estimation(self, frame, joints):
        """
        Determines the player's position and whether they are ready to throw.

        Args:
            frame (np.ndarray): Captured frame.
            joints (dict): Dictionary containing detected joint positions.

        Returns:
            str or None: One of the following movement statuses:
                - 'centered and throw identified'
                - 'centered'
                - 'move left'
                - 'move right'
                - None (if pose landmarks are missing)
        """
        if joints is None:# or 'body' not in joints or 'nose' not in joints:
            return None

        # Check for centering
        if all(element.visibility > 0.1 for element in joints['body']):
            body_position = sum(element.x for element in joints['body']) / len(joints['body'])
            
            if constants.CENTER - constants.POSE_TOLERANCE < body_position < constants.CENTER + constants.POSE_TOLERANCE:
                if any(element.y < joints['nose'].y for element in joints['right arm']) or \
                   any(element.y < joints['nose'].y for element in joints['left arm']):
                    return 'centered and throw identified'
                else:
                    return 'centered'
            else:
                direction = 'move left' if body_position < constants.CENTER else 'move right'
                return direction
        else:
            return None

    def cap_release(self):
        """
        Releases the camera resource.
        """
        if self.cap is not None:
            self.cap.release()


if __name__ == "__main__":

    cv = CV()

    while True:

        frame = cv.read_frame()
        frame_rgb, pose_results = cv.process_frame(frame)
        joints = cv.extract_joints(pose_results)

        # distance = cv.estimate_distance(frame, joints)
        distance = cv.smooth_distance(frame, joints)
        print(f'Distance: {distance}m')
        
        pose_estimation = cv.pose_estimation(frame, joints)
        print(f'Pose Estimation: {pose_estimation}')

        # Show the video feed with the landmarks
        cv2.imshow("Frisbeast Vision", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv.cap_release()
    cv2.destroyAllWindows()
