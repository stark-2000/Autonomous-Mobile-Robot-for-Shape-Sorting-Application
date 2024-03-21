import RPi.GPIO as gpio
import numpy as np
import time


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

#Initialize pwm signal to control motor
pwm1 = gpio.PWM(in1, 50)
val = 22
pwm1.start(val)

#Initialize pwm signal to control motor
pwm2 = gpio.PWM(in4, 50)
pwm2.start(val)
time.sleep(0.1)


with open("encodercontrol04.txt", "w") as f:
    for i in range(0, 300000):
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
        
        if counter1 >= 21:
            pwm1.stop()
        
        if counter2 >= 21:
            pwm2.stop()
        
        if counter1 >= 21 and counter2 >= 21:
            gameover()
            print("Thanks for playing!")
            break
