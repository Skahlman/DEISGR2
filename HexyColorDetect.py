# import the necessary packages
# from picamera2.array import PiRGBArray
from picamera2 import Picamera2, Preview
import time
import cv2
import numpy as np
# initialize the camera and grab a reference to the raw camera capture
#camera = PiCamera2()
#camera.resolution = (640, 480)
#camera.framerate = 32
picam = Picamera2()
camera_config = picam.create_preview_configuration()
picam.configure(camera_config)
# picam.start_preview(Preview.QTGL)
picam.start()
# rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)

lower_yellow = np.array([20, 50, 50], dtype="uint8")
upper_yellow = np.array([30, 255, 255], dtype="uint8")
cv2.namedWindow("trackbars")



ilowH = 11
ihighH = 53
ilowS = 56
ihighS = 135
ilowV = 186
ihighV = 255

def callback(x):
	pass
# create trackbars for color change
cv2.createTrackbar('lowH','trackbars',ilowH,179,callback)
cv2.createTrackbar('highH','trackbars',ihighH,179,callback)

cv2.createTrackbar('lowS','trackbars',ilowS,255,callback)
cv2.createTrackbar('highS','trackbars',ihighS,255,callback)

cv2.createTrackbar('lowV','trackbars',ilowV,255,callback)
cv2.createTrackbar('highV','trackbars',ihighV,255,callback)
# capture frames from the camera
while True:
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	frame = picam.capture_array()
	# show the frame
	hsv_image = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
	
	# yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
	
	ilowH = cv2.getTrackbarPos('lowH', 'trackbars')
	ihighH = cv2.getTrackbarPos('highH', 'trackbars')
	ilowS = cv2.getTrackbarPos('lowS', 'trackbars')
	ihighS = cv2.getTrackbarPos('highS', 'trackbars')
	ilowV = cv2.getTrackbarPos('lowV', 'trackbars')
	ihighV = cv2.getTrackbarPos('highV', 'trackbars')
	

	hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
	lower_hsv = np.array([ilowH, ilowS, ilowV])
	higher_hsv = np.array([ihighH, ihighS, ihighV])
	
	yellow_mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
	
	# filtering:
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
	yellow_mask = cv2.erode(yellow_mask,kernel,iterations = 2)
	yellow_mask = cv2.dilate(yellow_mask,kernel,iterations = 2)
	
	yellow_detected = cv2.bitwise_and(frame, frame, mask=yellow_mask)
	
	contours,_ = cv2.findContours(yellow_mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if contours:
		cv2.drawContours(frame,contours,-1,(0,255,0),2)
		cv2.drawContours(yellow_detected,contours,-1,(0,255,0),2)
		max_contour = max(contours,key=cv2.contourArea)
		M = cv2.moments(max_contour)
		if M["m00"] > 0:
			cX = int(M["m10"] / M["m00"] + 1e-5)
			cY = int(M["m01"] / M["m00"] + 1e-5)
			cv2.circle(frame,(cX,cY),5,(0,0,255),-1)
			nrows = frame.shape[0]
			ncols = frame.shape[1]
			frame_center_x = ncols / 2
			frame_center_y = nrows / 2
			# print(f"frame x = {frame_center_x}, frame y = {frame_center_y}")
			# distance = np.sqrt((cX - frame_center_x)**2 + (cY - frame_center_y)**2) 
			# distance = -distance if cX > frame_center_x else distance # distance is negative if object is on right side
			distance = cX - frame_center_x # horizontal distance 
			
			print("distance = ", distance)
			cv2.circle(frame,(int(frame_center_x),int(frame_center_y)),5,(255,0,0),-1)
			threshold = 20
			if abs(distance) <= threshold: # like in leage of legends
				print("move fwd")
			elif distance > 0:
				print("go left")
			else:
				print("go right")
	
	
	cv2.imshow("Yellow colour detection", yellow_detected)
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame
	# rawCapture.truncate(0)
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

picam.stop()
cv2.destroyAllWindows()
