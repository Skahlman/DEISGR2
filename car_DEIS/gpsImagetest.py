from __future__ import print_function
import cv2 as cv
import wget
import numpy as np
import urllib3

def download_image(url,save_as):
    http = urllib3.PoolManager()
    response = http.request('GET',url)
    with open(save_as, 'wb') as file:
        file.write(response.data)

url = 'http://192.168.1.2/axis-cgi/jpg/image.cgi'
save = 'image.jpg'

erode_filter = np.ones((9,9),np.uint8)
while True:
    download_image(url,save)
    img = cv.imread(save)
    frame_HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    #frame_threshold = cv.inRange(frame_HSV, (110, 80, 80), (130, 255, 255))
    frame_threshold = cv.inRange(frame_HSV, (0, 0, 0), (255, 255, 50))
    frame_threshold = cv.erode(frame_threshold,erode_filter)
    frame_threshold = cv.dilate(frame_threshold,erode_filter)
    frame_threshold = cv.circle(frame_threshold,(567,532),radius=10,color=(100,255,255),thickness=-1)
    frame_threshold = cv.circle(frame_threshold,(1000,1000),radius=10,color=(100,255,255),thickness=-1)
    cv.imshow("Threshold", frame_threshold)
    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break
cv.destroyAllWindows()
#filename = wget.download(url, out="image.jpg")


