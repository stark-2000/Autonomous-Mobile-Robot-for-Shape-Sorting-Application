import RPi.GPIO as GPIO

#Motor Driver Fins:
INA = 31
INB = 33
INC = 35
IND = 37

#Initialisation Function to set pins as  ip/op
def init():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    GPIO.setup(INC, GPIO.OUT)
    GPIO.setup(IND, GPIO.OUT)


#Set all motor driver pins to low
#(IN1, IN2, IN3, IN4)
def cleanup_all():
    GPIO.output(INA, GPIO.LOW)
    GPIO.output(INB, GPIO.LOW)
    GPIO.output(INC, GPIO.LOW)
    GPIO.output(IND, GPIO.LOW)

    GPIO.cleanup()

init()
cleanup_all()