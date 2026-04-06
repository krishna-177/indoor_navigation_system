import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class ArucoXYLocalization(Node):

    def __init__(self):
        super().__init__('aruco_xy_localization')

        self.sub = self.create_subscription(
            Image,
            '/ip_stream_image',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            Point,
            '/aruco/marker3_xy',
            10
        )

        self.bridge = CvBridge()

        # ArUco
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Marker info
        self.marker_length = 0.15  # meters (CHANGE to your marker size)

        # Camera calibration (REPLACE WITH YOUR VALUES)
        self.camera_matrix = np.array([
            [800, 0, 320],
            [0, 800, 240],
            [0, 0, 1]
        ], dtype=np.float32)

        self.dist_coeffs = np.zeros((5, 1))

        # Anchor IDs
        self.anchor_ids = [0, 1, 2]
        self.target_id = 3

        self.get_logger().info("ArUco XY localization node started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        if ids is None:
            return

        ids = ids.flatten()

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_length,
            self.camera_matrix,
            self.dist_coeffs
        )

        poses = {}
        for i, marker_id in enumerate(ids):
            poses[marker_id] = tvecs[i][0]  # (x,y,z)

        # Ensure all required markers are visible
        if not all(mid in poses for mid in self.anchor_ids + [self.target_id]):
            return

        # Use anchor ID 0 as origin
        origin = poses[0]
        target = poses[self.target_id]

        relative = target - origin

        # X–Y position in meters
        point = Point()
        point.x = float(relative[0])
        point.y = float(relative[1])
        point.z = 0.0

        self.pub.publish(point)

        self.get_logger().info(
            f"Marker 3 position -> x: {point.x:.2f} m, y: {point.y:.2f} m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoXYLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
