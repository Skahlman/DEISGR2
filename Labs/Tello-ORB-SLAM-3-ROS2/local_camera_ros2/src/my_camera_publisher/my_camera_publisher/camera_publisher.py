import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.publisher_ = self.create_publisher(Image, '/camera', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)  # Publish at ~10 Hz
        self.bridge = CvBridge()
        
        # Attempt to open the default camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera!')
        
    def timer_callback(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # Convert the frame to a ROS Image message
                msg = self.bridge.cv2_to_imgmsg(frame)
                self.publisher_.publish(msg)
            else:
                self.get_logger().warn('No frame received from camera.')

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down camera publisher node...')
    finally:
        if node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
