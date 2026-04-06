import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class ClickToWorld(Node):

    def __init__(self):
        super().__init__('click_to_world')

        # ROS I/O
        self.sub = self.create_subscription(
            Image,
            '/ip_stream_image',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            Point,
            '/clicked_goal_xy',
            10
        )

        self.bridge = CvBridge()

        # ArUco
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Square geometry (meters)
        self.world_points = {
            0: np.array([0.0, 0.0]),
            1: np.array([1.5, 0.0]),
            2: np.array([1.5, 1.5]),
            3: np.array([0.0, 1.5])
        }

        self.homography = None
        self.latest_frame = None

        # OpenCV window
        self.window_name = "Click to Select Goal"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info("Click-to-World node started")

    # ----------------------------------------------------

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.latest_frame = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            self.compute_homography(corners, ids)

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    # ----------------------------------------------------

    def compute_homography(self, corners, ids):
        image_pts = []
        world_pts = []

        for i, marker_id in enumerate(ids):
            if marker_id in self.world_points:
                center = corners[i][0].mean(axis=0)
                image_pts.append(center)
                world_pts.append(self.world_points[marker_id])

        if len(image_pts) == 4:
            image_pts = np.array(image_pts, dtype=np.float32)
            world_pts = np.array(world_pts, dtype=np.float32)

            self.homography, _ = cv2.findHomography(image_pts, world_pts)
            self.get_logger().info("Homography computed")

    # ----------------------------------------------------

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.homography is None:
            self.get_logger().warn("Homography not ready")
            return

        pixel = np.array([[x, y]], dtype=np.float32)
        pixel = np.array([pixel])

        world = cv2.perspectiveTransform(pixel, self.homography)[0][0]

        point = Point()
        point.x = float(world[0])
        point.y = float(world[1])
        point.z = 0.0

        self.pub.publish(point)

        self.get_logger().info(
            f"Clicked world point: x={point.x:.2f}, y={point.y:.2f}"
        )

        # Visual feedback
        cv2.circle(self.latest_frame, (x, y), 6, (0, 0, 255), -1)
        cv2.imshow(self.window_name, self.latest_frame)

# --------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ClickToWorld()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
