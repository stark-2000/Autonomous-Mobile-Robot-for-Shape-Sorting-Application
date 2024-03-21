import RPi.GPIO as GPIO
from time import sleep, time


class ultrasonic_range():
    def __init__(self):
        self.trig = 16
        self.echo = 18

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)


    def distance(self):
        #Ensure the trig pin is low  before starting
        GPIO.output(self.trig, False) #setting trig pin to 0
        sleep(0.01) 

        #creating a trig signal/pulse of 10us on-time
        GPIO.output(self.trig, True) #turning it on
        sleep(0.00001) #delay of 10us
        GPIO.output(self.trig, False) #turning it off

        pulse_start = 0
        pulse_end = 0

        #Measuring echo time signal
        while GPIO.input(self.echo) == 0: #when echo pin is low
            pulse_start = time() #store current time in pulse_start

        while GPIO.input(self.echo) == 1: #when echo pin is high
            pulse_end = time() #store current time in pulse_end

        pulse_duration = pulse_end - pulse_start #calc pulse duration in time
        distance = pulse_duration * 17150 #calc dist using speed of sound
        distance = round(distance, 2) #rounding off values to 2 decimal places
        return distance
	

p1 = ultrasonic_range()
def main():
    while True:
        dist = p1.distance()
        print("Distance: ", dist, " cm")

    GPIO.cleanup() #reset all the GPIO pins used


if __name__ == "__main__":
    main()