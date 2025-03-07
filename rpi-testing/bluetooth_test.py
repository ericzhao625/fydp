import threading
from bluedot.btcomm import BluetoothServer
from signal import pause
import time


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
        self.received_data = None
        self.running = True
        self.connected = False
        self.operation = None
        self.count = 0
        self.value = 0

        self.server = BluetoothServer(
            self.data_received_handler,
            port=1,
            when_client_connects=self.connect_handler,
            when_client_disconnects=self.disconnect_handler
        )

        self.task_thread = threading.Thread(target=self.background_task, daemon=True)
        self.task_thread.start()

    def connect_handler(self):
        """
        Runs when Bluetooth device connects to Raspberry Pi.
        """
        print('Client connected...')
        self.connected = True

    def disconnect_handler(self):
        """
        Runs when Bluetooth device disconnects from Raspberry Pi.
        """
        print('Client disconnected...')
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
        Dummy function for autonomous operation.
        """
        self.count += 1
        print(f"Autonomous mode running. Count: {self.count}")
        time.sleep(1)

    def manual(self):
        """
        Dummy function for manual operation.
        """
        if self.count < self.value:
            self.count += 1
            print(f"Manual mode running. Count: {self.count}")
            time.sleep(1)
        else:
            self.value = 0
            self.count = 0

    def background_task(self):
        """
        Runs a background loop while Bluetooth listens for data.
        """
        while self.running:

            # Check Bluetooth device is connected
            while self.connected:

                # Check if new data is received
                if self.received_data:
                    print(f"Processing: {self.received_data}")
                    
                    # Split data into list elements
                    self.processed_data = self.received_data.split(';')
                    
                    try:
                        # Autnomous operation
                        # Check if autonomous mode is turned on
                        if self.processed_data[0] == 'MODE:AUTO' and self.processed_data[1] == 'State:1':
                            self.operation = 'autonomous'
                            self.count = 0
                        # Check if autonomous mode is turned off
                        elif self.processed_data[0] == 'MODE:AUTO' and self.processed_data[1] == 'State:0':
                            self.operation = 'off'
                            self.count = 0
                        
                        # Manual operation
                        # Check if manual mode is selected
                        elif self.processed_data[0] == 'MODE:MANUAL':
                            self.operation = 'manual'
                            self.count = 0
                            print('manual running')

                        # Set throwing speed
                        elif self.processed_data[0].startswith('Speed'):
                            self.value = int(self.processed_data[0][6:])

                        # Adjust lateral aiming
                        elif self.processed_data[0] == 'Direction:Left':
                            print('We going left')
                        elif self.processed_data[0] == 'Direction:Right':
                            print('We going right')

                        # Adjust throwing tilt/height                        
                        elif self.processed_data[0] == 'Direction:Up':
                            print('We going right')
                        elif self.processed_data[0] == 'Direction:Down':
                            print('We going right')

                        # Turn off machine operation
                        elif self.processed_data[0] == 'MODE:OFF':
                            self.operation = 'off'

                    except Exception as e:
                        pass
                    
                    # Reset after processing
                    self.received_data = None
                    self.processed_data = None

                if self.operation == 'autonomous':
                    self.autonomous()
                
                elif self.operation == 'manual':
                    self.manual()

                # Prevent excessive CPU usage
                time.sleep(0.1)


if __name__ == '__main__':
    bluetooth = Bluetooth()
    pause()
