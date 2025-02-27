import RPi.GPIO as GPIO          
from time import sleep

in1 = 24
in2 = 23
en = 25
temp1=1

in3 = 17
in4 = 27
enb = 22
temp2 = 1

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
p=GPIO.PWM(en,1000)
p.start(25)

GPIO.setmode(GPIO.BCM)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(enb,GPIO.OUT)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
p2=GPIO.PWM(enb,1000)
p2.start(25)

print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

while(1):

    x=input()

    if x=='r':
        print("run")
        if(temp1==1):
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
        temp1=1
        x='z'

    elif x=='b':
        print("backward")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        temp1=0
        x='z'

    elif x=='l':
        print("low")
        p.ChangeDutyCycle(25)
        x='z'

    elif x=='m':
        print("medium")
        p.ChangeDutyCycle(50)
        x='z'

    elif x=='h':
        print("high")
        p.ChangeDutyCycle(75)
        x='z'
     
    
    elif x=='r2':
        print("run")
        if(temp1==1):
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
        temp1=1
        x='z'

    elif x=='b2':
        print("backward")
        GPIO.output(in3,GPIO.LOW)
        GPIO.output(in4,GPIO.HIGH)
        temp1=0
        x='z'

    elif x=='l2':
        print("low")
        p2.ChangeDutyCycle(25)
        x='z'

    elif x=='m2':
        print("medium")
        p2.ChangeDutyCycle(50)
        x='z'

    elif x=='h2':
        print("high")
        p2.ChangeDutyCycle(75)
        x='z'

    elif x=='e':
        GPIO.cleanup()
        break
    
    else:
        print("<<<  wrong data  >>>")
        print("please enter the defined data to continue.....")