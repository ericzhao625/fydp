import RPi.GPIO as GPIO          
from time import sleep

# Linear actuator 1
in1 = 14
in2 = 15
enable_a = 18
forward_1 = 1

# Linear actuator 2
in3 = 23
in4 = 24
enable_b = 25
forward_2 = 1

actuator_speed = 50

# Set up GPIO to control linear actuator 1
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(enable_a,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
p1 = GPIO.PWM(enable_a,1000)
p1.start(actuator_speed)

# Set up GPIO to control linear actuator 2
GPIO.setmode(GPIO.BCM)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(enable_b,GPIO.OUT)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
p2 = GPIO.PWM(enable_b,1000)
p2.start(actuator_speed)

print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward e-exit")
print("r2-run s2-stop f2-forward b2-backward e-exit")
print("\n")    

while(1):

    x=input()

    if x=='r':
        print("run")
        if(forward_1==1):
         GPIO.output(in1,GPIO.HIGH)
         GPIO.output(in2,GPIO.LOW)
         print("forward")
         x='z'
        else:
         GPIO.output(in1,GPIO.LOW)
         GPIO.output(in2,GPIO.HIGH)
         print("backward")
         x='z'


    elif x=='s':
        print("stop")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        x='z'

    elif x=='f':
        print("forward")
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        forward_1=1
        x='z'

    elif x=='b':
        print("backward")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        forward_1=0
        x='z'
    
    elif x=='r2':
        print("run")
        if(forward_1==1):
         GPIO.output(in3,GPIO.HIGH)
         GPIO.output(in4,GPIO.LOW)
         print("forward")
         x='z'
        else:
         GPIO.output(in3,GPIO.LOW)
         GPIO.output(in4,GPIO.HIGH)
         print("backward")
         x='z'


    elif x=='s2':
        print("stop")
        GPIO.output(in3,GPIO.LOW)
        GPIO.output(in4,GPIO.LOW)
        x='z'

    elif x=='f2':
        print("forward")
        GPIO.output(in3,GPIO.HIGH)
        GPIO.output(in4,GPIO.LOW)
        forward_2=1
        x='z'

    elif x=='b2':
        print("backward")
        GPIO.output(in3,GPIO.LOW)
        GPIO.output(in4,GPIO.HIGH)
        forward_2=0
        x='z'

    elif x=='e':
        GPIO.cleanup()
        break
    
    else:
        print("<<<  wrong data  >>>")
        print("please enter the defined data to continue.....")