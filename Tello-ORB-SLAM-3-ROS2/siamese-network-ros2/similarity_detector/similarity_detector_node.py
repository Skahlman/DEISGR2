import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import os
import cv2
import numpy as np
import tensorflow as tf
from cv_bridge import CvBridge

from similarity_detector.l1_distance_layer import L1DistanceLayer

class SimilarityDetectorNode(Node):
    def __init__(self):
        super().__init__('similarity_detector_node')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera')
        self.declare_parameter('model_path', 'resource/model')
        self.declare_parameter('reference_image', 'resource/reference.jpg')
        self.declare_parameter('similarity_threshold', 0.8)
        self.declare_parameter('publish_topic', '/detected_object')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        reference_image_path = self.get_parameter('reference_image').get_parameter_value().string_value
        self.similarity_threshold = self.get_parameter('similarity_threshold').get_parameter_value().double_value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value

        # Resolve full paths
        package_share = self.get_package_share_directory('similarity_detector')
        model_full_path = os.path.join(package_share, model_path)
        reference_full_path = os.path.join(package_share, reference_image_path)

        # Load model
        self.model = tf.keras.models.load_model(model_full_path, custom_objects={'L1DistanceLayer': L1DistanceLayer})
        self.get_logger().info(f"Model loaded from {model_full_path}")

        # Load and preprocess the reference image
        self.reference_embedding = self.compute_embedding(reference_full_path)
        self.get_logger().info(f"Reference image embedding computed.")

        self.bridge = CvBridge()

        # Subscriber to image topic
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # Publisher for detected object message
        self.publisher = self.create_publisher(String, publish_topic, 10)

    def get_package_share_directory(self, package_name: str) -> str:
        current_dir = os.path.dirname(os.path.realpath(__file__))
        return os.path.abspath(os.path.join(current_dir, '..'))

    def compute_embedding(self, image_path: str):
        # Load image from disk
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f"Could not load image at {image_path}")
            return None
        img_height, img_width = (64, 64)
        resized = cv2.resize(img, (img_width, img_height))
        input_image = resized.astype('float32') / 255.0
        input_image = np.expand_dims(input_image, axis=(0, -1))  # shape: (1, h, w, 1)

        # Compute the embedding using the model
        embedding = self.model.predict(input_image)  # assumes model returns embedding
        return embedding

    def image_callback(self, msg: Image):
        # Convert ROS Image to CV2/Numpy
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Preprocess
        img_height, img_width = (64, 64)  # Adjust as per model needs
        if len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = cv_image
        resized = cv2.resize(gray, (img_width, img_height))
        input_image = resized.astype('float32') / 255.0
        input_image = np.expand_dims(input_image, axis=(0, -1))  # shape: (1, h, w, 1)

        # Compute embedding of incoming image
        embedding = self.model.predict(input_image)

        # Compute similarity (here we use simple L2 distance; a lower distance means more similar)
        distance = np.linalg.norm(self.reference_embedding - embedding)
        self.get_logger().info(f"Distance to reference: {distance}")

        # If distance is below threshold, we consider it a match
        # Adjust logic as needed (lower distance = closer match)
        if distance < self.similarity_threshold:
            self.get_logger().info("Similar object detected!")
            msg = String(data="similar_object_found")
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimilarityDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
