import RPi.GPIO as GPIO          
from time import sleep

class HBridge:
    def __init__(self, in1, in2, enable, pwm_freq, pwm_dc):
        self.pin1 = in1
        self.pin2 = in2
        self.enable = enable
        self.pwm_freq = pwm_freq
        self.pwm_dc = pwm_dc

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin1,GPIO.OUT)
        GPIO.setup(self.pin2,GPIO.OUT)
        GPIO.setup(self.enable,GPIO.OUT)
        GPIO.output(self.pin1,GPIO.LOW)
        GPIO.output(self.pin2,GPIO.LOW)
        self.pwm = GPIO.PWM(self.enable, self.pwm_freq)
        self.pwm.start(self.pwm_dc)

    def forward(self):
        GPIO.output(self.pin1, GPIO.HIGH)
        GPIO.output(self.pin2, GPIO.LOW)

    def backward(self):
        GPIO.output(self.pin1,GPIO.LOW)
        GPIO.output(self.pin2,GPIO.HIGH)
        
    def stop(self):
        GPIO.output(self.pin1,GPIO.LOW)
        GPIO.output(self.pin2,GPIO.LOW)

    def stop_h_bridge(self):
        print('Stopping h bridge control')

        try:
            self.pwm.stop()
        except RuntimeError as e:
            print(f"Warning: GPIO cleanup failed - {e}")
        except Exception as e:
            print(f"Unexpected error during cleanup: {e}")

        print('H bridge control stopped')
