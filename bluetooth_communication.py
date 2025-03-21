import threading
from bluedot.btcomm import BluetoothServer
import cv2
import RPi.GPIO as GPIO
from signal import pause
import time

# from aiming import Aim
from cv import CV
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
    def __init__(self, frisbeast):
        """
        Initializes Bluetooth communication class.
        """
        # OpenCV window
        self.display = False

        # Initialize IMU
        # imu = IMU()

        # Initialize CV
        self.cv = CV()

        # Initialize throwing motor and solenoid
        self.frisbeast = frisbeast
        self.frisbeast.calibrate()
        # self.frisbeast.goal_roll = 16.5
        # self.frisbeast.goal_pitch = 0
        # self.frisbeast.activate_angle()
        self.motor_on_time = None

        # Initialize aiming
        # self.aim = Aim()

        self.received_data = None
        self.running = True
        self.connected = False
        self.operation = None
        self.command = None
        self.vertical = 0
        self.horizontal = 0
        self.speed = 0
        self.height = 0

        self.server = BluetoothServer(
            self.data_received_handler,
            port=1,
            when_client_connects=self.connect_handler,
            when_client_disconnects=self.disconnect_handler
        )
        self.server.read_size = 1024

        self.task_thread = threading.Thread(target=self.main, daemon=True)
        self.task_thread.start()

        # self.height = 178
        # self.cv.set_height(178)

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
        self.frisbeast.goal_roll = 16.5
        self.frisbeast.goal_pitch = 0
        self.frisbeast.activate_angle()
        self.frisbeast.yaw_active.set()
        # Capture frame
        frame, grab_time = self.cv.read_frame()
        # print(f"time between now and grab time: {time.time() - grab_time}")
        # Convert frame to RGB and get mediapipe output
        # frame_rgb, pose_results = self.cv.process_frame(frame)
        pose_results = self.cv.process_frame(frame)
        # Get the joints of interest
        joints = self.cv.extract_joints(pose_results)

        # Get player distance
        distance = self.cv.smooth_distance(frame, joints)
        # print(f'Distance: {distance}m')

        # Update throwing motor speed
        # if distance is not None:
        #     print(f"distance: {distance}")
        if joints is not None and distance is not None and 7.5 > distance > 3 and self.frisbeast.level.is_set():
            speed = distance / 7.5 * 100
            speed_diff = self.frisbeast.shooting_motor.speed - speed
            if speed_diff > 0:
                self.frisbeast.shooting_motor.forward(
                    min(
                        speed,
                        self.frisbeast.shooting_motor.speed + 10
                    )
                )
            else:
                self.frisbeast.shooting_motor.forward(speed)
            
            if self.motor_on_time is None:
                self.motor_on_time = time.time()
    
            # self.frisbeast.shooting_motor.forward(speed)
        else:
            self.frisbeast.shooting_motor.stop()
            self.motor_on_time = None

        # Get angle from center
        angle = self.cv.estimate_angle(frame, joints, distance)

        # Track player
        # if angle is not None:
            # if abs(angle) > 3:
        self.frisbeast.fix_angle(angle, grab_time)
                # time.sleep(0.1)

        # Get player pose
        pose_estimation = self.cv.pose_estimation(frame, joints, angle)
        # print(f'Pose Estimation: {pose_estimation}')

        # Release frisbee
        if self.motor_on_time is not None and time.time() - self.motor_on_time > 1:
            if self.frisbeast.push_frisbee(distance, pose_estimation):
                self.frisbeast.stop()
                self.operation = None
                time.sleep(1)

        # if self.display:
        #     # Display metrics
        #     # if distance is not None:
        #     #     display_metrics(frame, distance, distance / 15 * 100, pose_estimation)

        #     # Show the video feed with the landmarks
        #     cv2.imshow("Frisbeast Vision", frame)
        #     cv2.waitKey(1)

    def manual(self):
        """
        Function for manual operation.
        """
        self.frisbeast.yaw_active.clear()
        if self.speed > 2:
            self.frisbeast.shooting_motor.forward(self.speed)
        else:
            self.frisbeast.shooting_motor.stop()

        angle = self.frisbeast.imu.yaw
        if angle is None:
            return

        angle = (angle + 360) % 360
        if angle < self.frisbeast.origin:
            angle += 360

        diff = angle - self.frisbeast.origin - self.frisbeast.middle_angle
        # if abs(diff) > 10:


    
        if self.command == 'Direction:Left' and diff > -10:
            self.frisbeast.aiming_motor.left(50)
        
        elif self.command == 'Direction:Right' and diff < 10:
            self.frisbeast.aiming_motor.right(50)
        elif self.command == 'Direction:Reset':
            self.speed = 0
            self.vertical = 0
            self.horizontal = 0
            self.frisbeast.stop()
            self.frisbeast.home()
        
        else:
            self.frisbeast.aiming_motor.stop()
            # print('No turn')
        # self.aim.turn(self.command)

        # angle_changed = False
        # print("looping")
        self.frisbeast.goal_roll = self.vertical
        self.frisbeast.goal_pitch = self.horizontal
        self.frisbeast.activate_angle()
        # if self.vertical != self.frisbeast.goal_pitch:
        #     print(f"changing vertical to {self.vertical}")
        #     self.frisbeast.goal_pitch = self.vertical
        #     # angle_changed = True
        # if self.horizontal != self.frisbeast.goal_yaw:
        #     print(f"changing horizontal to {self.horizontal}")
        #     self.frisbeast.goal_yaw = self.horizontal
        #     # angle_changed = True
        # if angle_changed:
        #     print("setting angle")
        #     self.frisbeast.set_angle()
        if self.command == "Direction:Throw":
            self.frisbeast.push_frisbee(None, "MANUAL")

        self.command = None

    def process_data(self):
        """
        Process data and take command.
        """
        print(f"Processing: {self.received_data}")

        # Split data into list elements
        self.processed_data = self.received_data.split(';')
        if self.processed_data[-1] == "":
            data = self.processed_data[-2]
        else:
            data = self.processed_data[-1]
                    
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
            elif data.startswith('Speed'):
                self.speed = int(data[6:])

            # Set commands
            elif data.startswith('Direction:HorizontalAngle:'):
                print("processing horizontal")
                self.horizontal = float(data.split(":")[2])
                print(self.horizontal)
            
            elif data.startswith('Direction:VerticalAngle:'):
                print("processing vertical")
                self.vertical = float(data.split(":")[2])
                print(self.vertical)

            elif data.startswith('Direction'):
                self.command = data

            # Turn off machine operation
            elif data == 'MODE:OFF':
                self.operation = None

        except Exception as e:
            print(f"Bluetooth process exception: {e}")

    def operate(self):
        """
        Operate based on command.
        """
        # self.autonomous()
        if self.operation == 'autonomous':
            # start_time = time.time()
            # self.cv.active.set()
            self.autonomous()
            # print(f"autonomous time delay: {time.time() - start_time}")

        elif self.operation == 'manual':
            # print("loop manual")
            # self.cv.active.clear()
            self.manual()
            time.sleep(0.1)
        
        elif self.operation is None or self.operation == "off":
            # self.cv.active.clear()
            self.speed = 0
            self.frisbeast.yaw_active.clear()
            self.frisbeast.angle_active.clear()
            self.frisbeast.stop()
            self.frisbeast.shooting_motor.stop()
            # self.aim.stop()

    def cleanup(self):
        """
        Clean up resources.
        """
        # imu.cleanup()
        # self.throw.stop_motor()
        # self.aim.stop()
        self.frisbeast.stop()
        self.cv.cap_release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("Cleanup complete. Exiting safely.")

    def main(self):
        """
        Runs a background loop while Bluetooth listens for data.
        """
        while True:
            # self.operate()

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
                # time.sleep(0.1)


if __name__ == '__main__':
    bluetooth = Bluetooth()
    pause()
