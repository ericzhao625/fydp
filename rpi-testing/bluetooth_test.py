import threading
from bluedot.btcomm import BluetoothServer
from signal import pause
import time


class Bluetooth:
    def __init__(self):
        self.received_data = None
        self.running = True
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
        self.task_thread.start()  # Start background task in a separate thread

    def connect_handler(self):
        print('Client connected...')

    def disconnect_handler(self):
        print('Client disconnected...')

    def data_received_handler(self, data):
        """Handles incoming Bluetooth data."""
        self.received_data = data.strip()  # Store the latest received value
        print(f"Received: {self.received_data}")

        # Send acknowledgment back to client
        self.server.send(f"Acknowledged: {self.received_data}")

    def background_task(self):
        """Runs a background loop while Bluetooth listens for data."""
        while self.running:
            if self.received_data:  # Check if new data is received
                print(f"Processing: {self.received_data}")

                try:
                    if self.received_data == 'autonomous':
                        self.operation = 'autonomous'
                        self.count = 0
                    elif self.received_data == 'manual':
                        self.operation = 'manual'
                        self.count = 0

                    elif self.received_data.isdigit():
                        self.value = int(self.received_data)

                except Exception as e:
                    pass

                self.received_data = None  # Reset after processing

            if self.operation == 'autonomous':
                self.autonomous()
            
            elif self.operation == 'manual':
                self.manual()

            time.sleep(0.1)  # Prevent excessive CPU usage

    def autonomous(self):
        self.count += 1
        print(f"Autonomous mode running. Count: {self.count}")
        time.sleep(1)

    def manual(self):
        if self.count < self.value:
            self.count += 1
            print(f"Manual mode running. Count: {self.count}")
            time.sleep(1)
        else:
            self.value = 0
            self.count = 0


if __name__ == '__main__':
    bluetooth = Bluetooth()
    pause()
