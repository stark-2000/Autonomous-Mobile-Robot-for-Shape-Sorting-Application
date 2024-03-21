import RPi.GPIO as GPIO
from getkey import getkey, keys
from time import sleep


class teleop():
    def __init__(self):
        #Motor Driver Pins:
        self.INA = 31
        self.INB = 33
        self.INC = 35
        self.IND = 37

        #Set ip & op pins
        GPIO.setmode(GPIO.BOARD)
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
        #Left Wheels: (wheels rotating backward)
        GPIO.output(self.INA, GPIO.HIGH)
        GPIO.output(self.INB, GPIO.LOW)

        #Right Wheels: (wheels rotating forward)
        GPIO.output(self.INC, GPIO.HIGH)
        GPIO.output(self.IND, GPIO.LOW)

        sleep(delay) #wait x seconds
        self.stop() #stop the robot


    #Function to move the robot right
    def pivot_right(self, delay):
        #Left Wheels: (wheels rotating forward)
        GPIO.output(self.INA, GPIO.LOW)
        GPIO.output(self.INB, GPIO.HIGH)

        #Right Wheels: (wheels rotating backward)
        GPIO.output(self.INC, GPIO.LOW)
        GPIO.output(self.IND, GPIO.HIGH)

        sleep(delay) #wait x seconds
        self.stop() #stop the robot


p1 = teleop()

def main():
    print("Use Standard Gaming Controls to control the robot... ")
    while True:
        delay = 0.5 #delay between each movement
        while getkey() == keys.UP: #while the up arrow key is pressed
            p1.forward(delay)

        while getkey() == keys.DOWN: #while the down arrow key is pressed
            p1.backward(delay)

        while getkey() == keys.LEFT: #while the left arrow key is pressed
            p1.pivot_left(delay)

        while getkey() == keys.RIGHT: #while the right arrow key is pressed
            p1.pivot_right(delay)

        if getkey() == 'q': #if the 'q' key is pressed
            break

    GPIO.cleanup() #Clean all GPIO pin states


if __name__ == '__main__':
    main()