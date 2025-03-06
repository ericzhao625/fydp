from bluedot.btcomm import BluetoothServer
from signal import pause

def connect_handler():
    print('Client connected...')

def disconnect_handler():
    print('Client disconnected...')

def data_received_handler(data):
    print(data)
    # reverse text and send back to client
    s.send(data.rstrip()[::-1])

s = BluetoothServer(data_received_handler,
        port=1,
        when_client_connects=connect_handler,
        when_client_disconnects=disconnect_handler)

print('Listening...')
pause()