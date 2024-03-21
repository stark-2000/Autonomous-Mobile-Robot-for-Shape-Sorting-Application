import cv2
import numpy as np

def nothing(x):
    pass

# Create a window - one for GUI and one for output
cv2.namedWindow('GUI', cv2.WINDOW_NORMAL) #GUI window
cv2.resizeWindow('GUI', 640, 480) 

cv2.namedWindow('Masked Image multiplied with Original', cv2.WINDOW_NORMAL) #output window
cv2.resizeWindow('Masked Image multiplied with Original', 640, 480)

# Create trackbars for color change
# Hue is from 0-179 for Opencv
cv2.createTrackbar('HMin', 'GUI', 0, 179, nothing) #Hue min bar is from 0-179 for Opencv
cv2.createTrackbar('SMin', 'GUI', 0, 255, nothing) #Saturation min bar is from 0-255 for Opencv
cv2.createTrackbar('VMin', 'GUI', 0, 255, nothing) #Value min is from 0-255 for Opencv
cv2.createTrackbar('HMax', 'GUI', 0, 179, nothing) #Hue max bar is from 0-179 for Opencv 
cv2.createTrackbar('SMax', 'GUI', 0, 255, nothing) #Saturation max bar is from 0-255 for Opencv
cv2.createTrackbar('VMax', 'GUI', 0, 255, nothing) #Value max bar is from 0-255 for Opencv

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('HMax', 'GUI', 179) 
cv2.setTrackbarPos('SMax', 'GUI', 255)
cv2.setTrackbarPos('VMax', 'GUI', 255)

# Initialize HSV min/max values
hMin = sMin = vMin = hMax = sMax = vMax = 0

while(1):
    # Load image
    frame = cv2.imread("test2.jpg") #change the image name here - 22.png is a screenshot from the ball.mov video
    
    # Get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin', 'GUI') #get the current position of the Hue min bar
    sMin = cv2.getTrackbarPos('SMin', 'GUI') #get the current position of the Saturation min bar
    vMin = cv2.getTrackbarPos('VMin', 'GUI') #get the current position of the Value min bar
    hMax = cv2.getTrackbarPos('HMax', 'GUI') #get the current position of the Hue max bar
    sMax = cv2.getTrackbarPos('SMax', 'GUI') #get the current position of the Saturation max bar
    vMax = cv2.getTrackbarPos('VMax', 'GUI') #get the current position of the Value max bar

    # Set lower & upper bounds of HSV mask
    lower = np.array([hMin, sMin, vMin]) #create an array with the lower bounds of the HSV mask
    upper = np.array([hMax, sMax, vMax]) #create an array with the upper bounds of the HSV mask

    # Convert to HSV format, filter req color/object, and perform bitwise-AND
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #convert the image to HSV format
    mask = cv2.inRange(hsv, lower, upper) #create a mask with the lower and upper bounds
    result = cv2.bitwise_and(frame, frame, mask=mask) #perform bitwise-AND on the mask and the original image

    # Display result image
    cv2.imshow('Masked Image multiplied with Original', result) #display the result image which is the original image masked with the mask

    if cv2.waitKey(25) & 0xFF == ord('q'): #press q to exit
        break

cv2.destroyAllWindows()