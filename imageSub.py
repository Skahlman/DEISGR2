import numpy as np
import cv2 as cv
import lcm
import time

from exlcm import example_t

lc = lcm.LCM()


cap = cv.VideoCapture(0)
 
fgbg = cv.bgsegm.createBackgroundSubtractorMOG()
 
while(1):
    ret, frame = cap.read()
 
    fgmask = fgbg.apply(frame)
    
    if max(fgmask) > 0:
        msg = example_t()
        msg.timestamp = int(time.time() * 1000000)
        msg.position = (1, 2, 3)
        msg.orientation = (1, 0, 0, 0)
        msg.ranges = range(15)
        msg.num_ranges = len(msg.ranges)
        msg.name = "example string"
        msg.enabled = True
        lc.publish("EXAMPLE", msg.encode())
    else:
        
        
    cv.imshow('frame',fgmask)
    
    if cv.waitKey(30) & 0xFF == ord('q'):
            break
 
cap.release()
cv.destroyAllWindows()
