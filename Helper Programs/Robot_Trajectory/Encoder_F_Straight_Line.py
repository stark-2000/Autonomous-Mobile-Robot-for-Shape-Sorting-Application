import RPi.GPIO as gpio
import numpy as np


#Define pin allocations
in1 = 31
in2 = 33
in3 = 35
in4 = 37


right_encoder_pin = 12
left_encoder_pin = 7

#Initialize GPIO
def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(in1, gpio.OUT)
    gpio.setup(in2, gpio.OUT)
    gpio.setup(in3, gpio.OUT)
    gpio.setup(in4, gpio.OUT)

    gpio.setup(right_encoder_pin, gpio.IN, pull_up_down = gpio.PUD_UP)
    gpio.setup(left_encoder_pin, gpio.IN, pull_up_down = gpio.PUD_UP)


def gameover():
    gpio.output(in1, False)
    gpio.output(in2, False)
    gpio.output(in3, False)
    gpio.output(in4, False)

    gpio.cleanup()

#Main Code
init()

counter1 = np.uint64(0)
counter2  = np.uint64(0)

button1 = int(0)
button2 = int(0)


print("Enter Distance to Travel (in meter): ")
dist = input()
t_wheel_rev = 4.897 * float(dist)
ticks = t_wheel_rev * 21


pwm1 = gpio.PWM(in1, 50) #right side
pwm1.start(30)

pwm2 = gpio.PWM(in4, 50) #left side
pwm2.start(30)


with open("encodercontrol05.txt", "w") as f:
    while True:
        print("counter1 = ", counter1, "GPIO state 1: ", gpio.input(right_encoder_pin))
        print("counter2 = ", counter2, "GPIO state 2: ", gpio.input(left_encoder_pin))

        data = str(button1) + "," + str(button2) + '\n'
        f.write(data)

        if int(gpio.input(right_encoder_pin) != int(button1)):
            button1 = int(gpio.input(right_encoder_pin))
            counter1 += 1

        if int(gpio.input(left_encoder_pin) != int(button2)):
            button2 = int(gpio.input(left_encoder_pin))
            counter2 += 1

        if counter1 > counter2:
            pwm1.start(22)
            pwm2.start(34)
        
        if counter2 > counter1:
            pwm2.start(22)
            pwm1.start(34)

        if  ((counter1+counter2)/2) >= ticks:
            pwm1.stop()
            pwm2.stop()
            gameover()
            break