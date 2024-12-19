

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import cv2 as cv
#import wget
import numpy as np
import urllib3
import datetime

def download_image(url,save_as):
    http = urllib3.PoolManager()
    response = http.request('GET',url)
    with open(save_as, 'wb') as file:
        file.write(response.data)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'building_com', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

    def timer_callback(self):
        url = 'http://192.168.1.2/axis-cgi/jpg/image.cgi'
        save = 'image.jpg'
        save_hexy = '/home/deis-group2-rpi4/ros2_commands/src/py_pubsub/py_pubsub/hexy_image.jpg'
        radius = 400
        radius2 = 300
        center_x = 230
        center_y = 100

        erode_filter_hexy = np.ones((5,5),np.uint8)
        dilate_filter_hexy = np.ones((21,21),np.uint8)
        erode_filter2 = np.ones((15,15),np.uint8)
        download_image(url,save_hexy)
        img_hexy = cv.imread(save_hexy)
        frame_HSV_hexy = cv.cvtColor(img_hexy, cv.COLOR_BGR2HSV)
        frame_HSV_hexy = cv.GaussianBlur(frame_HSV_hexy,(5,5),cv.BORDER_DEFAULT)
        hexy_threshold = cv.inRange(frame_HSV_hexy, (110, 70, 90), (130, 255, 255))
        other_robot = cv.inRange(frame_HSV_hexy, (5,120,120), (15,255,255))
        #frame_threshold = cv.inRange(frame_HSV, (0, 0, 0), (255, 255, 60))
        hexy_threshold = cv.erode(hexy_threshold, erode_filter_hexy)
        other_robot = cv.erode(other_robot,erode_filter_hexy)
        hexy_threshold = cv.dilate(hexy_threshold, dilate_filter_hexy)
        other_robot = cv.dilate(other_robot,dilate_filter_hexy)
        hexy_threshold = cv.erode(hexy_threshold, erode_filter2)
        other_robot = cv.erode(other_robot,erode_filter2)
        #frame_threshold = cv.erode(frame_threshold,erode_filter)
        contours, hierarchies = cv.findContours(hexy_threshold, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        contours2,hierarchies2 = cv.findContours(other_robot, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

        for i in contours2:
            M = cv.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if ((cx-center_x)**2 + (cy-center_y)**2 <= radius2**2):
                    msg = String()
                    full_time = datetime.datetime.now()
                    date = full_time.strftime("%x")
                    time = full_time.strftime("%X")
                    microS = full_time.strftime("%f")
                    msg.data = f"data: {date}-{time}.{microS},dc,-1,-1,2,{cx};{cy}"
                    #msg.data = "HELP"
                    self.publisher_.publish(msg)
                    print("ORANGE")
        for i in contours:
            M = cv.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if ((cx-center_x)**2 + (cy-center_y)**2 <= radius**2):
                    msg = String()
                    full_time = datetime.datetime.now()
                    date = full_time.strftime("%x")
                    time = full_time.strftime("%X")
                    microS = full_time.strftime("%f") 
                    msg.data = f"data: {date}-{time}.{microS},dc,-1,-1,1,{cx};{cy}"
                    #msg.data = "HELP"
                    self.publisher_.publish(msg)
                    print("BLUE")

        
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    
    rclpy.spin(minimal_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

