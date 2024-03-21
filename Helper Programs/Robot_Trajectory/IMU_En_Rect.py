import RPi.GPIO as gpio
import numpy as np
import time
from pid import PID
import math
import serial #pyserial

#Define pin allocations
in1 = 31
in2 = 33
in3 = 35
in4 = 37

ser = serial.Serial('/dev/ttyUSB0', 9600) #Open serial port at 9600 baud

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


init()

def FB(dist):
    counter1 = np.uint64(0)
    counter2  = np.uint64(0)

    button1 = int(0)
    button2 = int(0)

    t_wheel_rev = 4.897 * float(dist)
    ticks = t_wheel_rev * 21

    pwm1 = gpio.PWM(in1, 50) #left side
    pwm1.start(30)

    pwm2 = gpio.PWM(in4, 50) #right side
    pwm2.start(30)

    val1 = int(1)
    val2 = int(1)

    while True:
        # print("counter1 = ", counter1, "GPIO state 1: ", gpio.input(right_encoder_pin))
        # print("counter2 = ", counter2, "GPIO state 2: ", gpio.input(left_encoder_pin))

        if int(gpio.input(right_encoder_pin) != int(button1)):
            button1 = int(gpio.input(right_encoder_pin))
            counter1 += 1

        if int(gpio.input(left_encoder_pin) != int(button2)):
            button2 = int(gpio.input(left_encoder_pin))
            counter2 += 1

        # if counter2 > counter1:
        #     val1 = val1 - 1
        #     val2 = val2 + 1

        if  counter1 >= ticks:
            pwm1.stop()
            pwm2.stop()
            gameover()
            break


def LR(angle):
    count = 0
    while(ser.in_waiting > 0): #Check if serial stream is available
        count += 1
        line = ser.readline() #Read the serial stream

        if count > 10: #Avoid first 10-lines of serial information
            line_filtered = line.rstrip().lstrip() #Strip serial stream of spaces
            line_filtered = str(line_filtered) #Convert serial stream to string
            line_filtered = line_filtered.strip("'") #Strip serial stream of '
            line = line_filtered.strip("b'") #Strip serial stream of b'

            line = float(line) #Convert serial stream to float
            print("Initial Angle: ",line,"\n") #Print the serial stream
            break

    pwm1 = gpio.PWM(in1, 50) #left side
    pwm1.start(40)

    pwm2 = gpio.PWM(in3, 50) #right side
    pwm2.start(40)

    while True:
        if(ser.in_waiting > 0): #Check if serial stream is available
            line1 = ser.readline() #Read the serial stream
            line_filtered1 = line1.rstrip().lstrip() #Strip serial stream of spaces
            
            line_filtered1 = str(line_filtered1) #Convert serial stream to string
            line_filtered1 = line_filtered1.strip("'") #Strip serial stream of '
            line1 = line_filtered1.strip("b'") #Strip serial stream of b'

            line1 = float(line1) #Convert serial stream to float
            print(line1,"\n") #Print the serial stream
            # print(abs(line1-line))
            if line < 90 and line1 > 330:
                line1 = line1 - 360 
    
       
        diff = abs(line1-line)
        if diff > angle - 2 and diff < angle + 2:
            print("Turning Stopped at: ", diff)
            pwm1.stop()
            pwm2.stop()
            gameover()
            break
            

            

print("Enter sequence of commands to follow: ")
print("Format: 'd,theta', d in meter, theta in degree")
print("Ex for rect path: 1,90,1,90,1,90,1,90 or 2,180,2,180,2,180,2,180 or 0.5,90,0.5,90,0.5,90,0.5,90 ")
data = input().split(',')

FB(float(data[0]))
LR(float(data[1]))

FB(float(data[2]))
LR(float(data[3]))

FB(float(data[4]))
LR(float(data[5]))

FB(float(data[6]))
LR(float(data[7]))