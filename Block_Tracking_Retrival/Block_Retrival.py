import cv2
import os
import numpy as np
import RPi.GPIO as gpio
from time import sleep
import serial
import math
from mail_smtp import mail_smtp



ser = serial.Serial('/dev/ttyUSB0', 9600) #Open serial port at 9600 baud

class block_pick():
    def __init__(self):
        self.h_object_real = 57
        self.h_sensor_px = 480
        self.h_sensor_mm = 2.76
        self.focal_len = 3.04

        self.in1 = 31
        self.in2 = 33
        self.in3 = 35
        self.in4 = 37

        self.right_encoder_pin = 12
        self.left_encoder_pin = 7

        gpio.setmode(gpio.BOARD)
        gpio.setup(self.in1, gpio.OUT)
        gpio.setup(self.in2, gpio.OUT)
        gpio.setup(self.in3, gpio.OUT)
        gpio.setup(self.in4, gpio.OUT)

        gpio.setup(self.right_encoder_pin, gpio.IN, pull_up_down = gpio.PUD_UP)
        gpio.setup(self.left_encoder_pin, gpio.IN, pull_up_down = gpio.PUD_UP)
        
        cv2.namedWindow('Output', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Output', 640, 480)

    def gameover(self):
        gpio.output(self.in1, False)
        gpio.output(self.in2, False)
        gpio.output(self.in3, False)
        gpio.output(self.in4, False)


    def FB(self,dist):
        print("Dist to go by FB fxn:(m) ", dist)
        counter1 = np.uint64(0)
        counter2  = np.uint64(0)

        button1 = int(0)
        button2 = int(0)

        t_wheel_rev = 4.897 * float(dist)
        ticks = t_wheel_rev * 21

        pwm1 = gpio.PWM(self.in1, 50) #left side
        pwm1.start(30)

        pwm2 = gpio.PWM(self.in4, 50) #right side
        pwm2.start(30)

        while True:
            # print("counter1 = ", counter1, "GPIO state 1: ", gpio.input(self.right_encoder_pin))
            # print("counter2 = ", counter2, "GPIO state 2: ", gpio.input(self.left_encoder_pin))

            if int(gpio.input(self.right_encoder_pin) != int(button1)):
                button1 = int(gpio.input(self.right_encoder_pin))
                counter1 += 1

            if int(gpio.input(self.left_encoder_pin) != int(button2)):
                button2 = int(gpio.input(self.left_encoder_pin))
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
                self.gameover()
                break

    
    def RB(self,dist):
        counter1 = np.uint64(0)
        counter2  = np.uint64(0)

        button1 = int(0)
        button2 = int(0)

        t_wheel_rev = 4.897 * float(dist)
        ticks = t_wheel_rev * 21

        pwm1 = gpio.PWM(self.in2, 50) #left side
        pwm1.start(30)

        pwm2 = gpio.PWM(self.in3, 50) #right side
        pwm2.start(30)

        while True:
            # print("counter1 = ", counter1, "GPIO state 1: ", gpio.input(self.right_encoder_pin))
            # print("counter2 = ", counter2, "GPIO state 2: ", gpio.input(self.left_encoder_pin))

            if int(gpio.input(self.right_encoder_pin) != int(button1)):
                button1 = int(gpio.input(self.right_encoder_pin))
                counter1 += 1

            if int(gpio.input(self.left_encoder_pin) != int(button2)):
                button2 = int(gpio.input(self.left_encoder_pin))
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
                self.gameover()
                break


    def turn(self,angle):
        counter1 = np.uint64(0)
        counter2  = np.uint64(0)

        button1 = int(0)
        button2 = int(0)

        BOT_RADIUS = 92.5
        WHEEL_RADIUS = 32.5

        bot_rot_peri = 2 * math.pi *BOT_RADIUS
        mm_per_degree = bot_rot_peri/360
        Ticks_per_mm = 20/(2* math.pi* WHEEL_RADIUS)
        Ticks_per_degree = mm_per_degree * Ticks_per_mm
        degree_per_tick = 1/Ticks_per_degree

        ticks = float(angle) * Ticks_per_degree
        
        pwm1 = gpio.PWM(in4, 50) #left side
        pwm1.start(80)

        pwm2 = gpio.PWM(in2, 50) #right side
        pwm2.start(80)      

        while True:
            # print("counter1 = ", counter1, "GPIO state 1: ", gpio.input(right_encoder_pin))
            # print("counter2 = ", counter2, "GPIO state 2: ", gpio.input(left_encoder_pin))

            if int(gpio.input(self.right_encoder_pin) != int(button1)):
                button1 = int(gpio.input(self.right_encoder_pin))
                counter1 += 1

            if int(gpio.input(self.left_encoder_pin) != int(button2)):
                button2 = int(gpio.input(self.left_encoder_pin))
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
                self.gameover()
                break


    def turn_IMU(self,angle):
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
                # print("Initial Angle: ",line,"\n") #Print the serial stream
                break

        pwm1 = gpio.PWM(self.in1, 50) #left side
        pwm1.start(60)

        pwm2 = gpio.PWM(self.in3, 50) #right side
        pwm2.start(60)

        while True:
            if(ser.in_waiting > 0): #Check if serial stream is available
                line1 = ser.readline() #Read the serial stream
                line_filtered1 = line1.rstrip().lstrip() #Strip serial stream of spaces
                
                line_filtered1 = str(line_filtered1) #Convert serial stream to string
                line_filtered1 = line_filtered1.strip("'") #Strip serial stream of '
                line1 = line_filtered1.strip("b'") #Strip serial stream of b'

                line1 = float(line1) #Convert serial stream to float
                # print(line1,"\n") #Print the serial stream
                # print(abs(line1-line))
                if line < 90 and line1 > 330:
                    line1 = line1 - 360 
        
            diff = abs(line1-line)
            if diff > angle - 2 and diff < angle + 2:
                # print("Turning Stopped at: ", diff)
                pwm1.stop()
                pwm2.stop()
                self.gameover()
                break



    def object_find(self):
        os.system("raspistill -o image.jpg -w 640 -h 480")
        img = cv2.imread("image.jpg")

        # Convert BGR to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        minHSV = np.array([152, 128, 0]) #figured out the threshold using GUI based color picker
        maxHSV = np.array([179, 255, 255]) #figured out the threshold using GUI based color picker

        maskhsv = cv2.inRange(hsv, minHSV, maxHSV)

        cont, _ = cv2.findContours(maskhsv.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if (len(cont) > 0):
            for i in range(len(cont)):
                area = cv2.contourArea(cont[i])
                if area > 1000:
                    print("Object found")
                    ret = True
                    (x,y),radius = cv2.minEnclosingCircle(cont[i])
                    cv2.circle(img, (int(x), int(y)), int(radius), (0, 0, 255), 2) #mark the circle
                    # image = cv2.resize(img, (0,0), fx=0.3, fy=0.3)
                    cv2.imshow("Output", img)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                    
                    area_circle = radius * radius * 3.14
                    print("Area of circle is: ", area_circle)

                    h_object_image_px = radius * 2
                    print("Image Height: ", h_object_image_px)

                    h_object_image_mm = (self.h_sensor_mm * h_object_image_px) / self.h_sensor_px
                    dist = (self.h_object_real * self.focal_len)/h_object_image_mm

                    print("distance to obstacle: ", dist)
                    return ret, dist-10


        print("No object found")
        ret = False
        return ret, 0


class servo_control():
    def __init__(self):
        servo_pin = 36

        gpio.setup(servo_pin, gpio.OUT)
        self.pwm = gpio.PWM(servo_pin,50)  #Set PWM frequency as 50Hz
                                    #Based on servo motor parameters
        self.pwm.start(3) #Starting PWM with dutycycle as 5%
                          #Some default value needed within bounds

    def servo_angle_set(self, dutycycle, delay): #dutycycle of servo, delay between each movement
        self.pwm.ChangeDutyCycle(dutycycle) #Change dutycycle of PWM
        sleep(delay) #Delay for given seconds
    
    def servo_power_down(self):
        self.pwm.ChangeDutyCycle(3) #Reset pos  of gripper to closed state
        sleep(1)
        self.pwm.stop() #Stop PWM


def main():
    p1 = block_pick()
    p2 = servo_control()
    p3 = mail_smtp()

    p2.servo_angle_set(7.5, 1) #5.5 duty cycle - servo at center (half open)

    ret, distance = p1.object_find()
    if ret:
        if distance > 0:
            print("Distance received:(cm) ", distance/10)
            p1.FB(distance/1000)
            print("Moved")
            p2.servo_angle_set(2.5, 1)
            print("Object Picked")
            p3.mail_snapshot('809T Robot Status - dhinesh - 119400241', 'Object Picked')

            p1.turn_IMU(90)
            print("Turned")
            
            p1.FB(distance/1000)
            print("Moved")

            p2.servo_angle_set(7.5, 1)
            print("Object Placed")
            p3.mail_snapshot('809T Robot Status - dhinesh - 119400241', 'Object Placed')

            p1.RB(0.3)
            print("Went Back")


        p2.servo_power_down() #Power down servo pwm and gpio

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()






if __name__ == "__main__":
    main()