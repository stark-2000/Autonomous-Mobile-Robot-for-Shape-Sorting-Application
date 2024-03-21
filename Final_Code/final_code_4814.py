#Dependencies:
import cv2
import numpy as np
import math

#RPi Dependencies:
import RPi.GPIO as gpio
from picamera import PiCamera
from time import sleep
import serial

from ultrasonic_range_test import ultrasonic_range
from mail_smtp import mail_smtp


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


class block_pick():
    def __init__(self):
        #Initialize the motor driver pins
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

        #Initialize the Raspberry Pi camera
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 25
        sleep(0.1) #Allow the camera to warmup

        self.h_object_real = 57 #Height of the object in real world in mm
        self.h_sensor_px = 480 #Height of the sensor in pixels (we take 480p images only)
        self.h_sensor_mm = 2.76 #Height of the sensor in mm (datasheet)
        self.focal_len = 3.04 #Focal length of the camera in mm (datasheet)
        self.dist_cam_to_acc = 145 #Dist between cam and IMU in mm

        self.ser = serial.Serial('/dev/ttyUSB0', 9600) #Open serial port at 9600 baud for IMU

        #Discard first 10 IMU values
        count = 0
        while True:
            if(self.ser.in_waiting > 0): #Check if serial stream is available
                count += 1
                line = self.ser.readline() #Read the serial stream

                if count > 10: #Avoid first 10-lines of serial information
                    print("Removed first 10 IMU junk values...\n")
                    break


    def gameover(self):
        gpio.output(self.in1, False)
        gpio.output(self.in2, False)
        gpio.output(self.in3, False)
        gpio.output(self.in4, False)

    
    currentIMUAngle = 0
    def get_IMU_angle(self):
        global currentIMUAngle
        if(self.ser.in_waiting > 0): #Check if serial stream is available
            line1 = self.ser.readline() #Read the serial stream
            line_filtered1 = line1.rstrip().lstrip() #Strip serial stream of spaces
            
            line_filtered1 = str(line_filtered1) #Convert serial stream to string
            line_filtered1 = line_filtered1.strip("'") #Strip serial stream of '
            line1 = line_filtered1.strip("b'") #Strip serial stream of b'

            line1 = float(line1) #Convert serial stream to float
            # print(line1,"\n") #Print the serial stream

            currentIMUAngle = line1
        return currentIMUAngle


    def robot_turn(self, angle, speed):
        #For Initial Value:
        error = angle - self.get_IMU_angle()
        error_continuous = error - 360*math.floor(0.5+error/360)

        if error_continuous > 0: #Turn left
            pwm1 = gpio.PWM(self.in1, 50) #left side
            pwm2 = gpio.PWM(self.in3, 50) #right side
            pwm1.start(speed)
            pwm2.start(speed)

            while True:
                #For consecutive measurements:
                error = angle - self.get_IMU_angle()
                error_continuous = error - 360*math.floor(0.5+error/360)
                if error_continuous < 0 or abs(error_continuous) < 1.0 :
                    print("Turning Stopped at: ", error_continuous)
                    pwm1.stop()
                    pwm2.stop()
                    self.gameover()
                    break

            
        if error_continuous < 0: #Turn right
            pwm3 = gpio.PWM(self.in2, 50) #left side 
            pwm4 = gpio.PWM(self.in4, 50) #right side
            pwm3.start(speed)
            pwm4.start(speed)

            while True:
                #For consecutive measurements:
                error = angle - self.get_IMU_angle()
                error_continuous = error - 360*math.floor(0.5+error/360)
                if error_continuous > 0 or abs(error_continuous) < 1.0 :
                    print("Turning Stopped at: ", error_continuous)
                    pwm3.stop()
                    pwm4.stop()
                    self.gameover()
                    break
            

    def robot_turn1(self, angle, speed):
        print("\nEntered 180")

        #For Initial Value:
        error = angle - self.get_IMU_angle()
        error_continuous = error - 360*math.floor(0.5+error/360)

        if error_continuous > 0: #Turn left
            pwm1 = gpio.PWM(self.in1, 50) #left side
            pwm2 = gpio.PWM(self.in3, 50) #right side
            pwm1.start(speed)
            pwm2.start(speed)

            while True:
                #For consecutive measurements:
                error = angle - self.get_IMU_angle()
                error_continuous = error - 360*math.floor(0.5+error/360)
                if error_continuous < 0 or abs(error_continuous) < 1.0 :
                    print("Turning Stopped at: ", error_continuous)
                    pwm1.stop()
                    pwm2.stop()
                    self.gameover()
                    break

            
        if error_continuous < 0: #Turn right
            pwm3 = gpio.PWM(self.in2, 50) #left side 
            pwm4 = gpio.PWM(self.in4, 50) #right side
            pwm3.start(speed)
            pwm4.start(speed)

            while True:
                #For consecutive measurements:
                error = angle - self.get_IMU_angle()
                error_continuous = error - 360*math.floor(0.5+error/360)
                if error_continuous > 0 or abs(error_continuous) < 1.0 :
                    print("Turning Stopped at: ", error_continuous)
                    pwm3.stop()
                    pwm4.stop()
                    self.gameover()
                    break


    def FB(self, dist, speed):
        # print("Dist to go printed from FB fxn:(m) ", dist)
        counter1 = np.uint64(0)
        counter2  = np.uint64(0)

        button1 = int(0)
        button2 = int(0)

        t_wheel_rev = 4.897 * float(dist)
        ticks = t_wheel_rev * 21

        pwm1 = gpio.PWM(self.in1, 50) #left side
        pwm1.start(speed)

        pwm2 = gpio.PWM(self.in4, 50) #right side
        pwm2.start(speed)

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
                pwm1.start(speed-8)
                pwm2.start(speed+4)
            
            if counter2 > counter1:
                pwm2.start(speed-8)
                pwm1.start(speed+4)

            if  ((counter1+counter2)/2) >= ticks:
                pwm1.stop()
                pwm2.stop()
                self.gameover()
                break


    def RB(self, dist, speed):
        counter1 = np.uint64(0)
        counter2  = np.uint64(0)

        button1 = int(0)
        button2 = int(0)

        t_wheel_rev = 4.897 * float(dist)
        ticks = t_wheel_rev * 21

        pwm1 = gpio.PWM(self.in2, 50) #left side
        pwm1.start(speed)

        pwm2 = gpio.PWM(self.in3, 50) #right side
        pwm2.start(speed)

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
                pwm1.start(speed-8)
                pwm2.start(speed+4)
            
            if counter2 > counter1:
                pwm2.start(speed-8)
                pwm1.start(speed+4)

            if  ((counter1+counter2)/2) >= ticks:
                pwm1.stop()
                pwm2.stop()
                self.gameover()
                break



    def red_object_find(self):
        #Capture frame from the camera
        self.camera.capture("img.png", use_video_port=False)
        img = cv2.imread("img.png")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #Convert BGR to HSVs

        minHSV = np.array([152, 128, 120]) 
        maxHSV = np.array([179, 255, 255]) 
        maskhsv = cv2.inRange(hsv, minHSV, maxHSV)

        cont, _ = cv2.findContours(maskhsv.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(cont) == 0:
            self.robot_turn(1, 50)
            print("Trying to find contours by rotating...")
            for i in range(0,90,20):
                self.robot_turn(i, 50)
                
                self.camera.capture("img.png", use_video_port=False)
                img = cv2.imread("img.png")
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #Convert BGR to HSV
                minHSV = np.array([152, 128, 120]) 
                maxHSV = np.array([179, 255, 255]) 
                maskhsv = cv2.inRange(hsv, minHSV, maxHSV)

                cont, _ = cv2.findContours(maskhsv.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

                if len(cont) > 0:
                    break

        if (len(cont) > 0):
            area = []
            for i in range(len(cont)):
                area.append(cv2.contourArea(cont[i]))

            max_area = max(area)
            i = area.index(max_area)
            if max_area > 0:
                print("\nRed Block Found...")
                (x,y),radius = cv2.minEnclosingCircle(cont[i])
                cv2.circle(img, (int(x), int(y)), int(radius), (0, 0, 255), 2) #mark the bounding circle
                cv2.circle(img, (int(x),int(y)), 2, (0, 0, 255), 2) #mark the center of the circle
                cv2.imwrite("img.png", img)
                # cv2.imshow("Frame", img)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()

                px_den = 62.2/640 #horizontal_fov/px = deg/px
                ang_obj = (x * px_den) - 31.1 #angle made by object with camera
                print("Object Angle with camera (deg): ", ang_obj)

                h_object_image_px = radius * 2
                h_object_image_mm = (self.h_sensor_mm * h_object_image_px) / self.h_sensor_px
                dist = (self.h_object_real * self.focal_len)/h_object_image_mm
                print("Distance to Obstacle (mm): ", dist) 
                print("\n")

                return dist, ang_obj


    
    def green_object_find(self):
        #Capture frame from the camera
        self.camera.capture("img.png", use_video_port=False)
        img = cv2.imread("img.png")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #Convert BGR to HSV

        minHSV = np.array([40, 0, 16]) 
        maxHSV = np.array([81, 255, 255]) 
        maskhsv = cv2.inRange(hsv, minHSV, maxHSV)

        cont, _ = cv2.findContours(maskhsv.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(cont) == 0:
            self.robot_turn(1, 50)
            print("Trying to find contours by rotating...")
            for i in range(0,90,20):
                self.robot_turn(i, 50)
                
                self.camera.capture("img.png", use_video_port=False)
                img = cv2.imread("img.png")
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #Convert BGR to HSV
                minHSV = np.array([40, 0, 16]) 
                maxHSV = np.array([81, 255, 255]) 
                maskhsv = cv2.inRange(hsv, minHSV, maxHSV)

                cont, _ = cv2.findContours(maskhsv.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

                if len(cont) > 0:
                    break


        if (len(cont) > 0):
            area = []
            for i in range(len(cont)):
                area.append(cv2.contourArea(cont[i]))

            max_area = max(area)
            i = area.index(max_area)
            if max_area > 0:
                print("\nGreen Block Found...")
                (x,y),radius = cv2.minEnclosingCircle(cont[i])
                cv2.circle(img, (int(x), int(y)), int(radius), (0, 0, 255), 2) #mark the bounding circle
                cv2.circle(img, (int(x),int(y)), 2, (0, 0, 255), 2) #mark the center of the circle
                cv2.imwrite("img.png", img)
                # cv2.imshow("Frame", img)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()

                px_den = 62.2/640 #horizontal_fov/px = deg/px
                ang_obj = (x * px_den) - 31.1 #angle made by object with camera
                print("Object Angle with camera (deg): ", ang_obj)

                h_object_image_px = radius * 2
                h_object_image_mm = (self.h_sensor_mm * h_object_image_px) / self.h_sensor_px
                dist = (self.h_object_real * self.focal_len)/h_object_image_mm
                print("Distance to Obstacle (mm): ", dist) 
                print("\n")

                return dist, ang_obj


    def blue_object_find(self):
        #Capture frame from the camera
        self.camera.capture("img.png", use_video_port=False)
        img = cv2.imread("img.png")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #Convert BGR to HSV

        minHSV = np.array([100, 96, 97]) 
        maxHSV = np.array([125, 255, 255]) 
        maskhsv = cv2.inRange(hsv, minHSV, maxHSV)

        cont, _ = cv2.findContours(maskhsv.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(cont) == 0:
            self.robot_turn(1, 50)
            print("Trying to find contours by rotating...")
            for i in range(0,90,20):
                self.robot_turn(i, 50)
                
                self.camera.capture("img.png", use_video_port=False)
                img = cv2.imread("img.png")
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #Convert BGR to HSV
                minHSV = np.array([100, 96, 97]) 
                maxHSV = np.array([125, 255, 255]) 
                maskhsv = cv2.inRange(hsv, minHSV, maxHSV)

                cont, _ = cv2.findContours(maskhsv.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

                if len(cont) > 0:
                    break


        if (len(cont) > 0):
            area = []
            for i in range(len(cont)):
                area.append(cv2.contourArea(cont[i]))

            max_area = max(area)
            i = area.index(max_area)
            if max_area > 0:
                print("\nBlue Block Found...")
                (x,y),radius = cv2.minEnclosingCircle(cont[i])
                cv2.circle(img, (int(x), int(y)), int(radius), (0, 0, 255), 2) #mark the bounding circle
                cv2.circle(img, (int(x),int(y)), 2, (0, 0, 255), 2) #mark the center of the circle
                cv2.imwrite("img.png", img)
                # cv2.imshow("Frame", img)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()

                px_den = 62.2/640 #horizontal_fov/px = deg/px
                ang_obj = (x * px_den) - 31.1 #angle made by object with camera
                print("Object Angle with camera (deg): ", ang_obj)

                h_object_image_px = radius * 2
                h_object_image_mm = (self.h_sensor_mm * h_object_image_px) / self.h_sensor_px
                dist = (self.h_object_real * self.focal_len)/h_object_image_mm
                print("Distance to Obstacle (mm): ", dist) 
                print("\n")

                return dist, ang_obj


p1 = block_pick()
p2 = servo_control()
p3 = ultrasonic_range()   
p4 = mail_smtp()
def set1_red():
    ### Red Block Picking & Placing:
    p2.servo_angle_set(3.5, 1)
    distance, angle = p1.red_object_find()
    angle = angle + p1.get_IMU_angle()
    p1.robot_turn(angle, 45)
    if distance > 3500:
        p1.FB((0.3*distance)/1000, 40)

    elif distance > 1000:
        p1.FB((0.5*distance)/1000, 40)

    else:
        p1.FB((0.2*distance)/1000, 40)

    # distance1, angle = p1.red_object_find()
    # distance2, angle = p1.red_object_find()
    # distance3, angle = p1.red_object_find()
    # distance = (distance1 + distance2 + distance3)/3
    # print("Mean Distance: ", distance)

    distance, angle = p1.red_object_find()
    angle = angle + p1.get_IMU_angle()
    print("Robot to turn by angle: (deg)", angle)
    p1.robot_turn(angle, 45)

    if distance > 200:
        p1.FB((distance*0.8)/1000, 40)

    p2.servo_angle_set(7.5, 1)

    # distance, angle = p1.red_object_find()
    # angle = angle + p1.get_IMU_angle()
    # print("Robot to turn by angle: (deg)", angle)
    # p1.robot_turn(angle, 45)
    # p1.FB((distance*0.95)/1000, 40)
    # p2.servo_angle_set(7.5, 1)

    distance, angle = p1.red_object_find()
    angle = angle + p1.get_IMU_angle()
    while distance > 150:
        p1.FB(25/1000, 30)
        p1.robot_turn(angle, 45)
        distance, angle = p1.red_object_find()
        print("Distance correction: ", distance)
        angle = angle + p1.get_IMU_angle()

    # angle = 2
    # count = 0
    # while (abs(angle) > 1.5):
    #     distance, angle = p1.red_object_find()
    #     angle1 = angle + p1.get_IMU_angle()
    #     print("Robot to turn by angle: (deg)", angle)
    #     p1.robot_turn(angle1, 40)
    #     count += 1

    #     if count > 5:
    #         break

    p1.FB((distance*0.9)/1000, 25)
    p2.servo_angle_set(3.5, 1)
    p4.mail_snapshot('809T Robot Status - dhinesh - 119400241', 'Red Block Picked', p1.camera)

    p1.robot_turn1(180, 60)
    
    p1.FB(300/1000, 50)
    while p3.distance() > 70:
        p1.FB(300/1000, 50)
    
    sleep(1)
    while p3.distance() > 70:
        p1.FB(50/1000, 50)

    p1.robot_turn(270, 60)

    p1.FB(200/1000, 50)
    while p3.distance() > 40:
        p1.FB(300/1000, 40)

    sleep(1)
    while p3.distance() > 40:
        p1.FB(50/1000, 40)

    p2.servo_angle_set(7.5, 1)
    p1.RB(0.15, 40)

    p1.robot_turn(45, 60)


def set2_red():
    ### Red Block Picking & Placing:
    p2.servo_angle_set(3.5, 1)
    distance, angle = p1.red_object_find()
    angle = angle + p1.get_IMU_angle()
    p1.robot_turn(angle, 45)
    if distance > 3500:
        p1.FB((0.3*distance)/1000, 40)

    elif distance > 1000:
        p1.FB((0.5*distance)/1000, 40)

    else:
        p1.FB((0.2*distance)/1000, 40)

    # distance1, angle = p1.red_object_find()
    # distance2, angle = p1.red_object_find()
    # distance3, angle = p1.red_object_find()
    # distance = (distance1 + distance2 + distance3)/3
    # print("Mean Distance: ", distance)

    distance, angle = p1.red_object_find()
    angle = angle + p1.get_IMU_angle()
    print("Robot to turn by angle: (deg)", angle)
    p1.robot_turn(angle, 45)

    if distance > 200:
        p1.FB((distance*0.8)/1000, 40)
    
    p2.servo_angle_set(7.5, 1)

    # distance, angle = p1.red_object_find()
    # angle = angle + p1.get_IMU_angle()
    # print("Robot to turn by angle: (deg)", angle)
    # p1.robot_turn(angle, 45)
    # p1.FB((distance*0.95)/1000, 40)
    # p2.servo_angle_set(7.5, 1)

    distance, angle = p1.red_object_find()
    angle = angle + p1.get_IMU_angle()
    while distance > 150:
        p1.FB(25/1000, 30)
        p1.robot_turn(angle, 45)
        distance, angle = p1.red_object_find()
        print("Distance correction: ", distance)
        angle = angle + p1.get_IMU_angle()

    # angle = 2
    # count = 0
    # while (abs(angle) > 1.5):
    #     distance, angle = p1.red_object_find()
    #     angle1 = angle + p1.get_IMU_angle()
    #     print("Robot to turn by angle: (deg)", angle)
    #     p1.robot_turn(angle1, 40)
    #     count += 1

    #     if count > 5:
    #         break
        
    p1.FB((distance*0.9)/1000, 25)
    p2.servo_angle_set(3.5, 1)
    p4.mail_snapshot('809T Robot Status - dhinesh - 119400241', 'Red Block Picked', p1.camera)

    p1.robot_turn(180, 60)

    p1.FB(300/1000, 50)
    while p3.distance() > 70:
        p1.FB(300/1000, 50)
    
    sleep(1)
    while p3.distance() > 70:
        p1.FB(50/1000, 50)

    p1.robot_turn(270, 60)

    p1.FB(200/1000, 50)
    while p3.distance() > 40:
        p1.FB(300/1000, 40)

    sleep(1)
    while p3.distance() > 40:
        p1.FB(50/1000, 40)

    p2.servo_angle_set(7.5, 1)
    p1.RB(0.15, 40)

    p1.robot_turn(45, 60)


def set_green():
    ### Green Block Picking & Placing:
    p2.servo_angle_set(3.5, 1)
    distance, angle = p1.green_object_find()
    angle = angle + p1.get_IMU_angle()
    p1.robot_turn(angle, 45)
    if distance > 3500:
        p1.FB((0.3*distance)/1000, 40)

    elif distance > 1000:
        p1.FB((0.5*distance)/1000, 40)

    else:
        p1.FB((0.2*distance)/1000, 40)

    # distance1, angle = p1.green_object_find()
    # distance2, angle = p1.green_object_find()
    # distance3, angle = p1.green_object_find()
    # distance = (distance1 + distance2 + distance3)/3
    # print("Mean Distance: ", distance)

    distance, angle = p1.green_object_find()
    angle = angle + p1.get_IMU_angle()
    print("Robot to turn by angle: (deg)", angle)
    p1.robot_turn(angle, 45)

    if distance > 200:
        p1.FB((distance*0.8)/1000, 40)
    
    p2.servo_angle_set(7.5, 1)

    # distance, angle = p1.green_object_find()
    # angle = angle + p1.get_IMU_angle()
    # print("Robot to turn by angle: (deg)", angle)
    # p1.robot_turn(angle, 45)
    # p1.FB((distance*0.95)/1000, 40)
    # p2.servo_angle_set(7.5, 1)

    distance, angle = p1.green_object_find()
    angle = angle + p1.get_IMU_angle()
    while distance > 150:
        p1.FB(25/1000, 30)
        p1.robot_turn(angle, 45)
        distance, angle = p1.green_object_find()
        print("Distance correction: ", distance)
        angle = angle + p1.get_IMU_angle()

    # angle = 2
    # count = 0
    # while (abs(angle) > 1.5):
    #     distance, angle = p1.green_object_find()
    #     angle1 = angle + p1.get_IMU_angle()
    #     print("Robot to turn by angle: (deg)", angle)
    #     p1.robot_turn(angle1, 40)
    #     count += 1

    #     if count > 5:
    #         break
        
    p1.FB((distance*0.9)/1000, 25)
    p2.servo_angle_set(3.5, 1)
    p4.mail_snapshot('809T Robot Status - dhinesh - 119400241', 'Green Block Picked', p1.camera)

    p1.robot_turn(180, 55)

    p1.FB(300/1000, 50)
    while p3.distance() > 70:
        p1.FB(300/1000, 50)
    
    sleep(1)
    while p3.distance() > 70:
        p1.FB(50/1000, 50)

    p1.robot_turn(270, 55)

    p1.FB(200/1000, 50)
    while p3.distance() > 40:
        p1.FB(300/1000, 40)

    sleep(1)
    while p3.distance() > 40:
        p1.FB(50/1000, 40)

    p2.servo_angle_set(7.5, 1)
    p1.RB(0.15, 40)

    p1.robot_turn(45, 55)


def set_blue():
    ### Blue Block Picking & Placing:
    p2.servo_angle_set(3.5, 1)
    distance, angle = p1.blue_object_find()
    angle = angle + p1.get_IMU_angle()
    p1.robot_turn(angle, 45)
    if distance > 3500:
        p1.FB((0.3*distance)/1000, 40)

    elif distance > 1000:
        p1.FB((0.5*distance)/1000, 40)

    else:
        p1.FB((0.2*distance)/1000, 40)

    # distance1, angle = p1.blue_object_find()
    # distance2, angle = p1.blue_object_find()
    # distance3, angle = p1.blue_object_find()
    # distance = (distance1 + distance2 + distance3)/3
    # print("Mean Distance: ", distance)

    distance, angle = p1.blue_object_find()
    angle = angle + p1.get_IMU_angle()
    print("Robot to turn by angle: (deg)", angle)
    p1.robot_turn(angle, 45)

    if distance > 200:
        p1.FB((distance*0.8)/1000, 40)
    
    p2.servo_angle_set(7.5, 1)

    # distance, angle = p1.blue_object_find()
    # angle = angle + p1.get_IMU_angle()
    # print("Robot to turn by angle: (deg)", angle)
    # p1.robot_turn(angle, 45)
    # p1.FB((distance*0.95)/1000, 40)
    # p2.servo_angle_set(7.5, 1)

    distance, angle = p1.blue_object_find()
    angle = angle + p1.get_IMU_angle()
    while distance > 150:
        p1.FB(25/1000, 30)
        p1.robot_turn(angle, 45)
        distance, angle = p1.blue_object_find()
        print("Distance correction: ", distance)
        angle = angle + p1.get_IMU_angle()

    # angle = 2
    # count = 0
    # while (abs(angle) > 1.5):
    #     distance, angle = p1.blue_object_find()
    #     angle1 = angle + p1.get_IMU_angle()
    #     print("Robot to turn by angle: (deg)", angle)
    #     p1.robot_turn(angle1, 40)
    #     count += 1

    #     if count > 5:
    #         break
        
    p1.FB((distance*0.9)/1000, 25)
    p2.servo_angle_set(3.5, 1)
    p4.mail_snapshot('809T Robot Status - dhinesh - 119400241', 'Blue Block Picked', p1.camera)

    p1.robot_turn(180, 55)

    p1.FB(300/1000, 50)
    while p3.distance() > 70:
        p1.FB(300/1000, 50)
    
    sleep(1)
    while p3.distance() > 70:
        p1.FB(50/1000, 50)

    p1.robot_turn(270, 55)

    p1.FB(200/1000, 50)
    while p3.distance() > 40:
        p1.FB(300/1000, 40)

    sleep(1)
    while p3.distance() > 40:
        p1.FB(50/1000, 40)

    p2.servo_angle_set(7.5, 1)
    p1.RB(0.15, 40)

    p1.robot_turn(45, 55)



def main():
    set1_red()
    set_green()
    set_blue()

    set2_red()
    set_green()
    set_blue()

    set2_red()
    set_green()
    set_blue()




if __name__ == "__main__":
    main()
    p4.close_email()
    gpio.cleanup()