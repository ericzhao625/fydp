# IMU
IMU_BUFFER_SIZE = 10

# Linear Actuators
LINEAR_ACTUATOR_FREQ = 100
LINEAR_ACTUATOR_DC = 50
# Linear Actuator 1
IN1 = 14
IN2 = 15
ENABLE_A = 18

# Linear Actuator 2
IN3 = 23
IN4 = 24
ENABLE_B = 25

# Linear Actuator 3
IN5 = 10
IN6 = 9
ENABLE_C = 11

# Aiming Motor
AIMING_MOTOR_FREQ = 100
AIMING_MOTOR_DC = 10
IN7 = 17
IN8 = 27
ENABLE_D = 22

# Throwing
# Motor
PWM_PIN = 12
PWM_FREQ = 500
PWM_INIT_DC = 0
MIN_DISTANCE = 5
MAX_DISTANCE = 36
MIN_PWM = 10
MAX_PWM = 50

# Solenoid
SOLENOID_PIN = 26
COOL_DOWN = 5

# Computer vision
CENTER = 0.5
POSE_TOLERANCE = 0.2
CV_BUFFER_SIZE = 10
CAMERA_FOCAL_LENGTH = 4 # mm
CAMERA_PIXEL_HEIGHT = 480
CAMERA_SENSOR_HEIGHT = 2.02    # mm
USER_HEIGHT = 1676  # mm
HEIGHT_TO_SHOULDERS = 0.87
