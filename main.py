from bluetooth_communication import Bluetooth


if __name__ == '__main__':
    bluetooth = Bluetooth()
    try:
        pause()
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected! Cleaning up resources.")

    finally:
        bluetooth.cleanup()

