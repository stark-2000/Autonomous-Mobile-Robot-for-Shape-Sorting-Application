#Dependencies:
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM) #Use GPIO no not pin no

from getkey import getkey, keys
from time import sleep, time
import numpy as np
from gpiozero import Servo
from ultrasonic_range_test import ultrasonic_range


class teleop():
    def __init__(self):
        #Motor Driver Pins:
        self.INA = 6 #31 pin no
        self.INB = 13 #33 pin no
        self.INC = 19 #35 pin no
        self.IND = 26 #37 pin no

        #Set ip & op pins
        GPIO.setup(self.INA, GPIO.OUT)
        GPIO.setup(self.INB, GPIO.OUT)
        GPIO.setup(self.INC, GPIO.OUT)
        GPIO.setup(self.IND, GPIO.OUT)


    #Function to stop the robot
    def stop(self):
        GPIO.output(self.INA, GPIO.LOW)
        GPIO.output(self.INB, GPIO.LOW)
        GPIO.output(self.INC, GPIO.LOW)
        GPIO.output(self.IND, GPIO.LOW)


    #Function to move the robot forward 
    def forward(self, delay):
        #Left Wheels: (wheels rotating forward)
        GPIO.output(self.INA, GPIO.HIGH)
        GPIO.output(self.INB, GPIO.LOW)

        #Right Wheels: (wheels rotating forward)
        GPIO.output(self.INC, GPIO.LOW)
        GPIO.output(self.IND, GPIO.HIGH)

        sleep(delay) #wait x seconds
        self.stop() #stop the robot


    #Function to move the robot backward 
    def backward(self, delay):
        #Left Wheels: (wheels rotating backward)
        GPIO.output(self.INA, GPIO.LOW)
        GPIO.output(self.INB, GPIO.HIGH)

        #Right Wheels: (wheels rotating backward)
        GPIO.output(self.INC, GPIO.HIGH)
        GPIO.output(self.IND, GPIO.LOW)

        sleep(delay) #wait x seconds
        self.stop() #stop the robot
   

    #Function to move the robot left
    def pivot_left(self, delay):
        #Left Wheels: (wheels rotating forward)
        GPIO.output(self.INA, GPIO.LOW)
        GPIO.output(self.INB, GPIO.HIGH)

        #Right Wheels: (wheels rotating backward)
        GPIO.output(self.INC, GPIO.LOW)
        GPIO.output(self.IND, GPIO.HIGH)
        
        sleep(delay) #wait x seconds
        self.stop() #stop the robot


    #Function to move the robot right
    def pivot_right(self, delay):
        #Left Wheels: (wheels rotating backward)
        GPIO.output(self.INA, GPIO.HIGH)
        GPIO.output(self.INB, GPIO.LOW)

        #Right Wheels: (wheels rotating forward)
        GPIO.output(self.INC, GPIO.HIGH)
        GPIO.output(self.IND, GPIO.LOW)

        sleep(delay) #wait x seconds
        self.stop() #stop the robot

    # def acce_dece(x):


class servo_control():
    def __init__(self):
        self.servo_pin = 16 #GPIO no as gpiozero lib uses BCM numbering
        self.servo = Servo(self.servo_pin, min_pulse_width=0.5/1000)
    
    def servo_power_down(self):
        self.servo.min()
        sleep(1)


p1 = teleop() 
p2 = servo_control()
p3 = ultrasonic_range()   

def main():
    print("Use Standard Gaming Controls to control the robot... ")
    while True:
        delay = 1
        while getkey() == keys.UP:
            p1.forward(delay)
            dist = p3.distance()
            print("Distance: ", dist, " cm")

        while getkey() == keys.DOWN:
            p1.backward(delay)
            dist = p3.distance()
            print("Distance: ", dist, " cm")

        while getkey() == keys.LEFT:
            p1.pivot_left(delay)
            dist = p3.distance()
            print("Distance: ", dist, " cm")

        while getkey() == keys.RIGHT:
            p1.pivot_right(delay)
            dist = p3.distance()
            print("Distance: ", dist, " cm")

        while getkey() == 'z':
            p2.servo.min()
            dist = p3.distance()
            print("Distance: ", dist, " cm")

        while getkey() == 'x':
            p2.servo.mid()
            dist = p3.distance()
            print("Distance: ", dist, " cm")
            
        while getkey() == 'c':
            p2.servo.max()
            dist = p3.distance()
            print("Distance: ", dist, " cm")

        if getkey() == 'q':
            break

    p2.servo_power_down() #Power down servo pwm & reset the pos of gripper
    GPIO.cleanup() #Clean all GPIO pin states


if __name__ == '__main__':
    main()