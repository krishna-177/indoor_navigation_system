import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class ArucoHomography(Node):

    def __init__(self):
        super().__init__('aruco_homography_localization')

        self.subscription = self.create_subscription(
            Image,
            '/ip_stream_image',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self.parameters = cv2.aruco.DetectorParameters_create()

        # Reference marker world coordinates (meters)
        self.world_points = {
            0: (0.0, 0.0),
            1: (1.5, 0.0),
            2: (0.0, 1.5),
            3: (1.5, 1.5)
        }

        self.reference_ids = [0,1,2,3]

        self.H = None

        self.get_logger().info("Homography localization node started")


    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters
        )

        if ids is None:
            return

        ids = ids.flatten()

        pixel_pts = []
        world_pts = []

        marker_centers = {}

        for i, marker_id in enumerate(ids):

            c = corners[i][0]
            center = c.mean(axis=0)

            marker_centers[marker_id] = center

            if marker_id in self.reference_ids:

                pixel_pts.append(center)
                world_pts.append(self.world_points[marker_id])

        # Compute homography
        if len(pixel_pts) >= 4:

            pixel_pts = np.array(pixel_pts, dtype=np.float32)
            world_pts = np.array(world_pts, dtype=np.float32)

            self.H, _ = cv2.findHomography(pixel_pts, world_pts)

        if self.H is None:
            return

        # Compute position of all other markers
        for marker_id, pixel in marker_centers.items():

            if marker_id not in self.reference_ids:

                p = np.array([pixel[0], pixel[1], 1.0])

                world = self.H @ p
                world = world / world[2]

                x = world[0]
                y = world[1]

                self.get_logger().info(
                    f"Marker {marker_id} -> x: {x:.2f} m , y: {y:.2f} m"
                )

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.imshow("Arena View", frame)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = ArucoHomography()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
