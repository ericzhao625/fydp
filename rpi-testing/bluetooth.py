from bluepy import btle

import secrets


class MyDelegate(btle.DefaultDelegate):
    def __init__(self):
        btle.DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):
        print("Received:", data.decode("utf-8"))

device = btle.Peripheral(secrets.BLUETOOTH_ID)
device.setDelegate(MyDelegate())

while True:
    if device.waitForNotifications(1.0):
        continue