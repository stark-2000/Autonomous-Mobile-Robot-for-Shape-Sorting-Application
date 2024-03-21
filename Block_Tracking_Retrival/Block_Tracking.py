import cv2
import os
import numpy as np
import RPi.GPIO as gpio
from time import sleep
import serial
import math

ser = serial.Serial('/dev/ttyUSB0', 9600) #Open serial port at 9600 baud

from picamera.array import PiRGBArray
from picamera import PiCamera
import time


# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
# allow the camera to warmup
time.sleep(0.1)

count = 0
while True:
    if(ser.in_waiting > 0): #Check if serial stream is available
        count += 1
        line = ser.readline() #Read the serial stream

        if count > 10: #Avoid first 10-lines of serial information
            line_filtered = line.rstrip().lstrip() #Strip serial stream of spaces
            line_filtered = str(line_filtered) #Convert serial stream to string
            line_filtered = line_filtered.strip("'") #Strip serial stream of '
            line = line_filtered.strip("b'") #Strip serial stream of b'

            line = float(line) #Convert serial stream to float
            # print("Initial Angle: ",line,"\n") #Print the serial stream
            print("Removed first 10 values")
            break



class block_pick():
    def __init__(self):
        self.in1 = 31
        self.in2 = 33
        self.in3 = 35
        self.in4 = 37

        self.h_object_real = 57
        self.h_sensor_px = 480
        self.h_sensor_mm = 2.76
        self.focal_len = 3.04

        gpio.setmode(gpio.BOARD)
        gpio.setup(self.in1, gpio.OUT)
        gpio.setup(self.in2, gpio.OUT)
        gpio.setup(self.in3, gpio.OUT)
        gpio.setup(self.in4, gpio.OUT)
        
        # cv2.namedWindow('Output', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('Output', 640, 480)

    def gameover(self):
        gpio.output(self.in1, False)
        gpio.output(self.in2, False)
        gpio.output(self.in3, False)
        gpio.output(self.in4, False)

    
    def turn_IMU(self,angle):
        
        pwm1 = gpio.PWM(self.in1, 40) #left side
        pwm1.start(40)

        pwm2 = gpio.PWM(self.in3, 40) #right side
        pwm2.start(40)

        pwm3 = gpio.PWM(self.in2, 40)
        pwm4 = gpio.PWM(self.in4, 40)

        if(ser.in_waiting > 0): #Check if serial stream is available
            print("Got Initial Value")
            line = ser.readline() #Read the serial stream
            line_filtered = line.rstrip().lstrip() #Strip serial stream of spaces
            
            line_filtered = str(line_filtered) #Convert serial stream to string
            line_filtered = line_filtered.strip("'") #Strip serial stream of '
            line = line_filtered.strip("b'") #Strip serial stream of b'

            line = float(line) #Convert serial stream to float

            error1 = angle - line
            error_continuous1 = error1 - 360*math.floor(0.5+error1/360)
            
            if error_continuous1 < 0:
                pwm1.start(40)
                pwm2.start(40)
            
            else:
                pwm1.stop()
                pwm2.stop()
                pwm3.start(40)
                pwm4.start(40)


            while True:
                if(ser.in_waiting > 0): #Check if serial stream is available
                    # print("Entered IMU Loop")
                    line1 = ser.readline() #Read the serial stream
                    line_filtered1 = line1.rstrip().lstrip() #Strip serial stream of spaces
                    
                    line_filtered1 = str(line_filtered1) #Convert serial stream to string
                    line_filtered1 = line_filtered1.strip("'") #Strip serial stream of '
                    line1 = line_filtered1.strip("b'") #Strip serial stream of b'

                    line1 = float(line1) #Convert serial stream to float
                    # print(line1,"\n") #Print the serial stream
                    # print(abs(line1-line))
                    
                    error = angle - line1
                    error_continuous = error - 360*math.floor(0.5+error/360)
                    print(error_continuous)

                    if (error_continuous > -1 and error_continuous < 1) or ((error_continuous * error_continuous1) < 0):
                        print("Turning Stopped at: ", line1 - line)
                        pwm1.stop()
                        pwm2.stop()
                        pwm3.stop()
                        pwm4.stop()
                        self.gameover()
                        break



    def object_find(self):
        # keep looping
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
            # grab the current frame
            img = frame.array
            # show the frame to our screen
            # cv2.imshow("Frame", img)
            
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
                        (x,y),radius = cv2.minEnclosingCircle(cont[i])
                        cv2.circle(img, (int(x), int(y)), int(radius), (0, 0, 255), 2) #mark the circle
                        cv2.circle(img, (int(x),int(y)), 2, (0, 0, 255), 2) 
                        cv2.imshow("Frame", img)
                        cv2.waitKey(0)
                        cv2.destroyAllWindows()

                        px_den = 62.2/640
                        lin_dist = x * px_den
                        print("Lin dist:(mm) ", lin_dist)

                        area_circle = radius * radius * 3.14
                        print("Area of circle is: ", area_circle)

                        h_object_image_px = radius * 2
                        print("Image Height: ", h_object_image_px)

                        h_object_image_mm = (self.h_sensor_mm * h_object_image_px) / self.h_sensor_px
                        dist = (self.h_object_real * self.focal_len)/h_object_image_mm
                        print("distance to obstacle: ", dist)

                        theta_to_move = lin_dist - 31.1

                        # theta_to_move = math.asin(lin_dist/dist)
                        # theta_to_move = theta_to_move * 180/math.pi
                        print("Correction Angle: ", theta_to_move)

                        if (not math.isnan(theta_to_move)):
                            self.turn_IMU(theta_to_move)    

                break

            
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            
            # press the 'q' key to stop the video stream
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break





def main():
    p1 = block_pick()
    p1.object_find()

    # cv2.destroyAllWindows()





if __name__ == "__main__":
    main()