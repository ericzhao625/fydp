# import cv2
# import numpy as np
# import mediapipe as mp

# def pose_detection(video_frame):
#     image_rgb = cv2.cvtColor(video_frame, cv2.COLOR_BGR2RGB)

#     results = pose.process(image_rgb)

#     if results.pose_process(image_rgb):

#         mp.solutions.drawing_utils.draw_landmarks(video_frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

#         nose = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]

#         print(nose.x)

# if __name__ == '__main__':

#     # print(cv2.__version__)

#     cap = cv2.VideoCapture(0)

#     cap.set(cv2.CAP_PROP_FPS, 5)
#     fps = cap.get(cv2.CAP_PROP_FPS)
#     print(f'Camera FPS: {fps}')

#     # mp_pose = mp.solutions.pose
#     # pose = mp_pose.Pose()

#     while cap.isOpened():
#         ret, frame = cap.read()

#         if not ret:
#             break

#         # pose_detection(frame)

#         cv2.imshow('Frisbee', frame)

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()

# from picamera2 import Picamera2
# import cv2
# import numpy as np

# print('pre')
# # Initialize the camera
# camera = Picamera2()
# print('post')
# # Set the resolution
# camera.configure(camera.create_still_configuration())
# camera.set_controls({'Brightness': 65})

# # Start the camera preview
# camera.start()

# # Set up a window to display the image
# while True:
#     # Capture a frame
#     frame = camera.capture_array()

#     # Convert to BGR for OpenCV (picamera2 captures in RGB)
#     img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

#     # Display the image
#     cv2.imshow("Original Image", img)

#     # Break the loop if 'q' is pressed
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Clean up
# cv2.destroyAllWindows()
# camera.stop()

# from picamera2 import Picamera2

# picam2 = Picamera2()
# picam2.start()
# picam2.capture_file("image.jpg")
# picam2.stop()

# import cv2
# from picamera2 import Picamera2
# import time

# print('initialize')
# picam2 = Picamera2()
# print('configure')
# picam2.configure(picam2.create_preview_configuration(raw={'size': (1640, 1232)}, main={'format': 'RGB888', 'size': (640, 480)}))
# print('start')
# picam2.start()
# print('sleep')
# time.sleep(2)

# img = cv2.imread('apples.jpeg')

# print('loop')
# # while True:
# print('in loop')
# # img = picam2.capture_array()

# print('display')
# cv2.imshow('Output', img)
# print('check interrupt')
# cv2.waitKey(0)

# # if cv2.waitKey(1) & 0xFF == ord('q'):
# #     break

# print('stop')
# picam2.stop()
# picam2.close()

# import cv2

# img = cv2.imread('apples.jpeg')

# cv2.imshow("Output", img)
# cv2.waitKey(0)


# import cv2
# print(cv2.__version__)

# cam = cv2.VideoCapture(0)

# while True:
#     (ret, frame) = cam.read()

#     cv2.imshow('Preview', frame)

#     if cv2.waitKey(0) & 0xFF == ord('q'):
#         break

# import cv2
# from picamera2 import Picamera2
# import time

# print('initialize')
# picam2 = Picamera2()
# print('configure')
# picam2.configure(picam2.create_preview_configuration(raw={'size': (1640, 1232)}, main={'format': 'RGB888', 'size': (640, 480)}))
# print('start')
# picam2.start()
# print('sleep')
# time.sleep(2)

# print('loop')
# while True:
#     print('in loop')
#     # Capture an image from the camera
#     img = picam2.capture_array()

#     # Debug the shape and type of img
#     print(f"Image shape: {img.shape}, Image dtype: {img.dtype}")

#     # Check if img is in the correct format
#     if img is None:
#         print("Error: captured image is None.")
#         break

#     print('display')
#     cv2.imshow('Output', img)

#     # Wait for a key press (with a small timeout to avoid freezing)
#     print('check interrupt')
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# print('stop')
# picam2.stop()
# picam2.close()

# cv2.destroyAllWindows()

# import cv2
# import numpy as np

# # Create a simple image (black)
# img = np.zeros((480, 640, 3), dtype=np.uint8)

# cv2.imshow('Output', img)
# cv2.waitKey(0)  # Wait for a key press
# cv2.destroyAllWindows()


# # Credit: Adrian Rosebrock
# # https://www.pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/
 
# # import the necessary packages
# from picamera.array import PiRGBArray # Generates a 3D RGB array
# from picamera import PiCamera # Provides a Python interface for the RPi Camera Module
# import time # Provides time-related functions
# import cv2 # OpenCV library
 
# # Initialize the camera
# camera = PiCamera()
 
# # Set the camera resolution
# camera.resolution = (640, 480)
 
# # Set the number of frames per second
# camera.framerate = 32
 
# # Generates a 3D RGB array and stores it in rawCapture
# raw_capture = PiRGBArray(camera, size=(640, 480))
 
# # Wait a certain number of seconds to allow the camera time to warmup
# time.sleep(0.1)
 
# # Capture frames continuously from the camera
# for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
     
#     # Grab the raw NumPy array representing the image
#     image = frame.array
     
#     # Display the frame using OpenCV
#     cv2.imshow("Frame", image)
     
#     # Wait for keyPress for 1 millisecond
#     key = cv2.waitKey(1) & 0xFF
     
#     # Clear the stream in preparation for the next frame
#     raw_capture.truncate(0)
     
#     # If the `q` key was pressed, break from the loop
#     if key == ord("q"):
#         break

#!/usr/bin/python3

# import cv2

# from picamera2 import Picamera2

# # Grab images as numpy arrays and leave everything else to OpenCV.

# # face_detector = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
# cv2.startWindowThread()

# # picam2 = Picamera2()
# # picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
# # picam2.start()

# # while True:
# # im = picam2.capture_array()
# im = cv2.imread('apples.jpeg')

# # grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
# # faces = face_detector.detectMultiScale(grey, 1.1, 5)

# # for (x, y, w, h) in faces:
# #     cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0))

# cv2.imshow("Camera", im)
# cv2.waitKey(1)

# from picamera2 import Picamera2, Preview
# import time
# picam2 = Picamera2()
# camera_config = picam2.create_still_configuration(main={'size': (1920, 1080)}, lores={'size': (640, 480)}, display='lores')
# picam2.configure(camera_config)
# picam2.start_preview(Preview.QTGL)
# time.sleep(2)
# picam2.start()
# time.sleep(2)
# picam2.capture_file('test.jpg')

# from picamera2 import Picamera2, Preview
# import time
# picam2 = Picamera2()
# camera_config = picam2.create_preview_configuration(main={'size': (1920, 1080)}, lores={'size': (640, 480)}, display='lores')
# picam2.configure(camera_config)
# picam2.start_preview(Preview.QTGL)
# time.sleep(2)
# picam2.start()
# time.sleep(2)
# picam2.capture_file('test.jpg')

# import matplotlib.pyplot as plt
# from picamera2 import Picamera2
# import cv2

# picam2 = Picamera2()
# picam2.configure(picam2.create_preview_configuration(main={'format': 'RGB888', 'size': (640, 480)}))
# picam2.start()

# frame = picam2.capture_array()
# plt.imshow(frame)
# # print(frame.shape, frame.dtype)
# # print(frame.max(), frame.min())
# # image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

# # cv2.imshow('Frame', image)
# picam2.stop()

# while True:
#     print('pre')
#     frame = picam2.capture_array()
#     print(type(frame))

#     image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

#     cv2.imshow('Frame', image)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# picam2.stop()
# cv2.destroyAllWindows()

# import cv2
# import numpy as np
# test_image = np.zeros((480, 640, 3), dtype=np.uint8)
# cv2.imshow('test image', test_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# from picamera2 import Picamera2
# import cv2

# # picam2 = Picamera2()
# # picam2.configure(picam2.create_preview_configuration(main={'format': 'RGB888', 'size': (640, 480)}))
# # picam2.start()

# # frame = picam2.capture_file('test.jpg')
# cv2.imshow('image', cv2.imread('test.jpg'))
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# import numpy as np
# import cv2

# # cap = cv2.VideoCapture('/dev/video0')

# camera = cv2.VideoCapture(0)
# camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

# while True:
#     ret, frame = camera.read()
#     cv2.imshow('frame', frame)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# camera.release()
# cv2.destroyAllWindows()

# import matplotlib.pyplot as plt
# import cv2
# from picamera2 import Picamera2
# import numpy as np

# picam2 = Picamera2()
# picam2.preview_configuration.main.size = (1920, 1080)
# picam2.preview_configuration.main.format = 'RGB888'
# picam2.start()

# im = picam2.capture_array()
# print(im.shape, im.dtype)
# print(np.min(im), np.max(im))
# # plt.imshow(im)
# # plt.show()

# # im = picam2.capture_file('test.jpg')
# # cv2.imshow('image', cv2.imread('test.jpg'))
# # cv2.waitKey(0)
# # cv2.destroyAllWindows()

# while True:
#     im = picam2.capture_array()
#     image_rgb = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
#     print('pre')
#     cv2.imshow('preview', image_rgb)
#     print(image_rgb.shape)
#     # if cv2.waitKey(1) == ord('q'):
#     #     break
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

# picam2.stop()
# # cv2.destroyAllWindows()

# import cv2
# import numpy as np
# test_image = np.zeros((480, 640, 3), dtype=np.uint8)
# test_image[:, :, 1] = 255
# cv2.imshow('test image', test_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# from picamera2 import Picamera2
# import cv2

# picam2 = Picamera2()
# picam2.configure(picam2.create_preview_configuration(main={'format': 'RGB888', 'size': (640, 480)}))
# picam2.start()

# while True:
#     frame = picam2.capture_array()

#     if frame is None:
#         print('Failed to capture frame')
#         break
    
#     print(frame.shape, frame.dtype)

#     frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

#     cv2.imshow('camera feed', frame_bgr)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

#     cv2.waitKey(10)

# picam2.stop()
# cv2.destroyAllWindows()

# import cv2
# import numpy as np
# # from picamera2 import Picamera2

# # cam = Picamera2()
# height = 480
# width = 640
# middle = (int(width / 2), int(height / 2))
# # cam.configure(cam.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}))

# # cam.start()
# test_image = np.zeros((480, 640, 3), dtype=np.uint8)
# test_image[:, :, 1] = 255
# # while True:
#     # frame = cam.capture_array()
#     # cv2.circle(frame, middle, 10, (255, 0 , 255), -1)
# cv2.imshow('f', cv2.imread('apples.jpeg'))
# cv2.waitKey(0)

# # if cv2.waitKey(1) & 0xFF == ord('q'):
# #     break

# # cam.stop()
# cv2.destroyAllWindows()


import cv2
import numpy as np
import os
import mediapipe as mp
import psutil
import time
import RPi.GPIO as GPIO
from time import sleep

pinLED = 3


def monitor_memory():

    process = psutil.Process(os.getpid())

    memory_usage = process.memory_info().rss / (1024 * 1024)  # Memory in MB
    memory_usage = f"Memory Usage: {memory_usage:.2f} MB"
    return memory_usage


def calculate_length(joint_1, joint_2):
    joint_1 = np.array([joint_1.x, joint_1.y, joint_1.z])
    joint_2 = np.array([joint_2.x, joint_2.y, joint_2.z])

    joint_2_to_1 = joint_1 - joint_2

    length = np.linalg.norm(joint_2_to_1)

    return length


def pose_detection(video_frame):

    # Convert the image from BGR to RGB
    image_rgb = cv2.cvtColor(video_frame, cv2.COLOR_BGR2RGB)

    # Process the image and get pose landmarks
    results = pose.process(image_rgb)

    if results.pose_landmarks:
        # Draw the landmarks on the image
        # mp.solutions.drawing_utils.draw_landmarks(video_frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        body = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER],
                results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER],
                results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP],
                results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP])

        # Check for throw signal
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
            if center - pose_tolerance < body_position < center + pose_tolerance:
                cv2.putText(video_frame, 'POSE: centered enough', (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                if all(element.visibility > 0.1 for element in left_arm) or all(element.visibility > 0.1 for element in right_arm):
                    if any(element.y < nose.y for element in left_arm) or any(element.y < nose.y for element in right_arm):
                        cv2.putText(video_frame, 'POSE: throw signal identified', (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        GPIO.output(pinLED, GPIO.HIGH)

                    else:
                        GPIO.output(pinLED, GPIO.LOW)

            elif body_position < center + pose_tolerance:
                cv2.putText(video_frame, 'POSE: move left', (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                GPIO.output(pinLED, GPIO.LOW)

            elif body_position > center - pose_tolerance:
                cv2.putText(video_frame, 'POSE: move right', (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                GPIO.output(pinLED, GPIO.LOW)

        else:
            cv2.putText(video_frame, "POSE: can't find torso landmarks for person", (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            GPIO.output(pinLED, GPIO.LOW)


if __name__ == "__main__":
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

        pose_detection(frame)

        cv2.putText(frame, monitor_memory(), (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show the video feed with the landmarks
        cv2.imshow("Pose Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()