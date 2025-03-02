import cv2
import numpy as np
import os
import mediapipe as mp
import psutil
import time
import RPi.GPIO as GPIO
import constants


def monitor_memory():
    """
    Monitor memory usage from CV system

    :return: memory usage
    """

    process = psutil.Process(os.getpid())

    memory_usage = process.memory_info().rss / (1024 * 1024)  # Memory in MB
    memory_usage = f"Memory Usage: {memory_usage:.2f} MB"
    return memory_usage


def calculate_length(joint_1, joint_2):
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


def pose_detection(mp_pose, pose, video_frame):
    """
    Track the player and perform pose detection to detect throw signal.
    """

    distance_to_object = None

    # Convert the image from BGR to RGB
    image_rgb = cv2.cvtColor(video_frame, cv2.COLOR_BGR2RGB)

    # Process the image and get pose landmarks
    results = pose.process(image_rgb)

    if results.pose_landmarks:
        body = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER],
                results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER],
                results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP],
                results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP])

        left_arm = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST],
                    results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW])
        right_arm = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST],
                     results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW])
        nose = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]

        length = calculate_length(right_arm[0], right_arm[1])
        distance_to_object = (4 * 280 * 720) / (length * 480 * 2.02) / 1000
        cv2.putText(video_frame, f'Estimated Distance: {distance_to_object:.2f}', (0, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Check for centering
        if all(element.visibility > 0.1 for element in body):
            body_position = sum(element.x for element in body) / len(body)
            if constants.CENTER - constants.POSE_TOLERANCE < body_position < constants.CENTER + constants.POSE_TOLERANCE:
                cv2.putText(video_frame, 'POSE: centered enough', (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                if any(element.y < nose.y for element in left_arm) or any(element.y < nose.y for element in right_arm):
                    cv2.putText(video_frame, 'POSE: throw signal identified', (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    GPIO.output(constants.PIN_LED, GPIO.HIGH)
                else:
                    GPIO.output(constants.PIN_LED, GPIO.LOW)
            else:
                direction = 'move left' if body_position < constants.CENTER else 'move right'
                cv2.putText(video_frame, f'POSE: {direction}', (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                GPIO.output(constants.PIN_LED, GPIO.LOW)
        else:
            cv2.putText(video_frame, "POSE: can't find torso landmarks for person", (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            GPIO.output(constants.PIN_LED, GPIO.LOW)

    return distance_to_object


if __name__ == "__main__":
    pinLED = 17
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(pinLED, GPIO.OUT)

    # MediaPipe
    center = 0.5
    pose_tolerance = 0.2

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

        pose_detection(mp_pose, pose, frame)

        # Display memory usage
        # cv2.putText(frame, monitor_memory(), (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show the video feed with the landmarks
        cv2.imshow("Pose Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()