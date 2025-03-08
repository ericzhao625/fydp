import threading
from bluedot.btcomm import BluetoothServer
import cv2
import RPi.GPIO as GPIO
from signal import pause
import time

from aiming import Aim
from cv import CV
from imu import IMU
from throwing import Throw
from utils import display_metrics

class Bluetooth:
    """
    Bluetooth class for communication between Raspberry Pi and Android device.

    Attributes:
        received_data (string or None): String input from Bluetooth device.
        running (Boolean): Boolean to run program.
        connected (Boolean): Boolean to store connectivity.
        operation (string or None): Command sent from Bluetooth device.
        count (int): Count for testing.
        value (int): Value to count to.
    """
    def __init__(self):
        """
        Initializes Bluetooth communication class.
        """
        # OpenCV window
        self.display = True

        # Initialize IMU
        # imu = IMU()

        # Initialize CV
        self.cv = CV()

        # Initialize throwing motor and solenoid
        self.throw = Throw()

        # Initialize aiming
        self.aim = Aim()

        self.received_data = None
        self.running = True
        self.connected = False
        self.operation = None
        self.command = None
        self.speed = 0
        self.height = 0

        self.server = BluetoothServer(
            self.data_received_handler,
            port=1,
            when_client_connects=self.connect_handler,
            when_client_disconnects=self.disconnect_handler
        )

        self.task_thread = threading.Thread(target=self.main, daemon=True)
        self.task_thread.start()

    def connect_handler(self):
        """
        Runs when Bluetooth device connects to Raspberry Pi.
        """
        print('Device connected')
        self.connected = True

    def disconnect_handler(self):
        """
        Runs when Bluetooth device disconnects from Raspberry Pi.
        """
        print('Device disconnected')
        self.connected = False

    def data_received_handler(self, data):
        """
        Handles incoming Bluetooth data.
        """
        # Store the latest received value
        self.received_data = data.strip()
        print(f"Received: {self.received_data}")

        # Send acknowledgment back to client
        self.server.send(f"Acknowledged: {self.received_data}")

    def autonomous(self):
        """
        Function for autonomous operation.
        """
        # Capture frame
        frame = self.cv.read_frame()
        # Convert frame to RGB and get mediapipe output
        frame_rgb, pose_results = self.cv.process_frame(frame)
        # Get the joints of interest
        joints = self.cv.extract_joints(pose_results)

        # Get player distance
        distance = self.cv.smooth_distance(frame, joints)
        # print(f'Distance: {distance}m')

        # Update throwing motor speed
        pwm_value = self.throw.update_motor_speed(distance)

        # Get angle from center
        angle = self.cv.estimate_angle(frame, joints, distance)

        # Track player
        self.aim.track_player(angle)

        # Get player pose
        pose_estimation = self.cv.pose_estimation(frame, joints, angle)
        # print(f'Pose Estimation: {pose_estimation}')

        # Release frisbee
        self.throw.push_frisbee(distance, pose_estimation)

        if self.display:
            # Display metrics
            display_metrics(frame, distance, pwm_value, pose_estimation)

            # Show the video feed with the landmarks
            cv2.imshow("Frisbeast Vision", frame)
            cv2.waitKey(1)

    def manual(self):
        """
        Function for manual operation.
        """
        self.throw.update_manual_motor_speed(self.speed)
        self.aim.turn(self.command)
        self.throw.push_frisbee(self.throw.max_distance, self.command)

        self.command = None

    def process_data(self):
        """
        Process data and take command.
        """
        print(f"Processing: {self.received_data}")

        # Split data into list elements
        self.processed_data = self.received_data.split(';')
                    
        try:
            # Autnomous operation
            # Check if autonomous mode is turned on
            if self.processed_data[0] == 'MODE:AUTO' and self.processed_data[1] == 'State:1' and self.processed_data[2].startswith('Height:'):
                self.operation = 'autonomous'
                self.height = int(self.processed_data[2][7:])
                self.cv.set_height(self.height)

            # Check if autonomous mode is turned off
            elif self.processed_data[0] == 'MODE:AUTO' and self.processed_data[1] == 'State:0':
                self.operation = 'off'
            
            # Manual operation
            # Check if manual mode is selected
            elif self.processed_data[0] == 'MODE:MANUAL':
                self.operation = 'manual'
                print('manual running')

            # Set throwing speed
            elif self.processed_data[0].startswith('Speed'):
                self.speed = int(self.processed_data[0][6:])

            # Set commands
            elif self.processed_data[0].startswith('Direction'):
                self.command = self.processed_data[0]
                print(self.command)

            # Turn off machine operation
            elif self.processed_data[0] == 'MODE:OFF':
                self.operation = None

        except Exception as e:
            pass

    def operate(self):
        """
        Operate based on command.
        """
        if self.operation == 'autonomous':
            self.autonomous()

        elif self.operation == 'manual':
            self.manual()
        
        elif self.operation is None:
            self.throw.update_manual_motor_speed(0)
            self.aim.stop()

    def cleanup(self):
        """
        Clean up resources.
        """
        # imu.cleanup()
        self.throw.stop_motor()
        self.aim.stop_h_bridge()
        self.cv.cap_release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("Cleanup complete. Exiting safely.")

    def main(self):
        """
        Runs a background loop while Bluetooth listens for data.
        """
        while True:

            # Check Bluetooth device is connected
            if self.connected:

                # Check if new data is received
                if self.received_data:
                
                    # Process data
                    self.process_data()
                    
                    # Reset after processing
                    self.received_data = None
                    self.processed_data = None

                # Operate based on command
                self.operate()

                # Prevent excessive CPU usage
                time.sleep(0.1)


if __name__ == '__main__':
    bluetooth = Bluetooth()
    pause()
