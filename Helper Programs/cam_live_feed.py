import cv2
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

# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('videoname.avi', fourcc, 10, (640, 480))


# keep looping
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
	# grab the current frame
	image = frame.array
	# show the frame to our screen
	cv2.imshow("Frame", image)
	
	# write frame to video file
	out.write(image)
	
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	
	# press the 'q' key to stop the video stream
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break
