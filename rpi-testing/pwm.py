import RPi.GPIO as GPIO
import time

# Pin configuration
PWM_PIN = 13  # Physical pin 33

# Setup
GPIO.setmode(GPIO.BCM)  # Use Broadcom GPIO pin numbering
GPIO.setup(PWM_PIN, GPIO.OUT)

# # Initialize PWM on the pin with a frequency of 100 Hz
# pwm = GPIO.PWM(PWM_PIN, 500)
# pwm.start(75)  # Start PWM with 0% duty cycle
# Initialize PWM on the pin with a frequency of 100 Hz
# pwm = GPIO.PWM(PWM_PIN, 500)
# pwm.start(75)  # Start PWM with 0% duty cycle
# Initialize PWM on the pin with a frequency of 100 Hz
pwm = GPIO.PWM(PWM_PIN, 500)
pwm.start(20)  # Start PWM with 0% duty cycle

try:
    while True:
        time.sleep(0.05)
        # # Gradually increase brightness
        # for duty_cycle in range(0, 100, 1):  # 0 to 100% in steps of 1
        #     pwm.ChangeDutyCycle(duty_cycle)
        #     print(duty_cycle)
        #     time.sleep(0.05)  # Wait 50ms

        # # Gradually decrease brightness
        # for duty_cycle in range(100, -1, -1):  # 100 to 0% in steps of -1
        #     pwm.ChangeDutyCycle(duty_cycle)
        #     print(duty_cycle)
        #     time.sleep(0.05)  # Wait 50ms

except KeyboardInterrupt:
    # Exit the loop when Ctrl+C is pressed
    print("Exiting gracefully")

finally:
    # Clean up
    pwm.stop()  # Stop PWM
    GPIO.cleanup()  # Reset GPIO settings
    print("GPIO cleanup completed")
