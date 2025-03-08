import RPi.GPIO as GPIO
import cv2

from aiming import Aim
from bluetooth_communication import Bluetooth
from cv import CV
from imu import IMU
from signal import pause
from throwing import Throw
from utils import display_metrics


if __name__ == '__main__':
    bluetooth = Bluetooth()
    pause()
