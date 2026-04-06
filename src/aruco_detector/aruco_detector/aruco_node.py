import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        # Subscriber: IP camera stream
        self.image_sub = self.create_subscription(
            Image,
            '/ip_stream_image',   # change if needed
            self.image_callback,
            10
        )

        # Publishers
        self.image_pub = self.create_publisher(
            Image,
            '/aruco/image_annotated',
            10
        )

        self.id_pub = self.create_publisher(
            Int32MultiArray,
            '/aruco/ids',
            10
        )

        self.bridge = CvBridge()

        # ArUco setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self.parameters = cv2.aruco.DetectorParameters_create()


        self.get_logger().info('ArUco detector node started')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters
        )

        id_msg = Int32MultiArray()

        if ids is not None:
            ids = ids.flatten()
            id_msg.data = ids.tolist()

            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Publish detected IDs
        self.id_pub.publish(id_msg)

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

