import RPi.GPIO as GPIO
import cv2

from aiming import Aim
from cv import CV
from imu import IMU
from throwing import Throw
from utils import display_metrics


if __name__ == '__main__':
    # OpenCV window
    display = True

    # Initialize IMU
    # imu = IMU()

    # Initialize CV
    cv = CV()

    # Initialize throwing motor and solenoid
    throw = Throw()

    # Initialize aiming
    aim = Aim()

    try:
        while True:
            # Get IMU readings
            # yaw, pitch, roll = imu.smooth_readings()

            # Capture frame
            frame = cv.read_frame()
            # Convert frame to RGB and get mediapipe output
            frame_rgb, pose_results = cv.process_frame(frame)
            # Get the joints of interest
            joints = cv.extract_joints(pose_results)

            # Get player distance
            distance = cv.smooth_distance(frame, joints)
            # print(f'Distance: {distance}m')

            # Update throwing motor speed
            pwm_value = throw.update_motor_speed(distance)

            # Get angle from center
            angle = cv.estimate_angle(frame, joints, distance)

            # Track player
            aim.track_player(angle)

            # Get player pose
            pose_estimation = cv.pose_estimation(frame, joints, angle)
            # print(f'Pose Estimation: {pose_estimation}')

            # Release frisbee
            throw.push_frisbee(distance, pose_estimation)

            if display:
                # Display metrics
                display_metrics(frame, distance, pwm_value, pose_estimation)

                # Show the video feed with the landmarks
                cv2.imshow("Frisbeast Vision", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("Keyboard Interrupt detected! Cleaning up resources.")

    finally:
        # Cleanup resources
        # imu.cleanup()
        throw.stop_motor()
        aim.stop_h_bridge()
        cv.cap_release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("Cleanup complete. Exiting safely.")
