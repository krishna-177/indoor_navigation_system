import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class ReferenceClickLocalization(Node):

    def __init__(self):
        super().__init__('reference_click_localization')

        # ROS
        self.sub = self.create_subscription(
            Image,
            '/ip_stream_image',
            self.image_callback,
            10
        )

        self.goal_pub = self.create_publisher(Point, '/clicked_goal_xy', 10)
        self.robot_pub = self.create_publisher(Point, '/robot_xy', 10)

        self.bridge = CvBridge()

        # ArUco
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Marker size (meters)
        self.marker_length = 0.15  # CHANGE if needed

        # Camera calibration (REPLACE with real values)
        self.camera_matrix = np.array([
            [800, 0, 320],
            [0, 800, 240],
            [0, 0, 1]
        ], dtype=np.float32)

        self.dist_coeffs = np.zeros((5, 1))

        # Reference frame storage
        self.ref_origin = None
        self.ref_x = None
        self.ref_y = None

        # OpenCV
        self.window = "Reference Click Localization"
        cv2.namedWindow(self.window)
        cv2.setMouseCallback(self.window, self.mouse_callback)

        self.latest_frame = None
        self.poses = {}

        self.get_logger().info("Reference Click Localization Node Started")

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

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs
            )

            self.poses.clear()
            for i, marker_id in enumerate(ids):
                self.poses[marker_id] = tvecs[i][0]

            self.build_reference_frame()
            self.publish_robot_position()

        cv2.imshow(self.window, frame)
        cv2.waitKey(1)

    # ----------------------------------------------------

    def build_reference_frame(self):
        if not all(mid in self.poses for mid in [0, 1, 2]):
            return

        origin = self.poses[0]

        x_axis = self.poses[1] - origin
        y_axis = self.poses[2] - origin

        self.ref_x = x_axis / np.linalg.norm(x_axis)
        self.ref_y = y_axis / np.linalg.norm(y_axis)
        self.ref_origin = origin

    # ----------------------------------------------------

    def publish_robot_position(self):
        if self.ref_origin is None or 3 not in self.poses:
            return

        rel = self.poses[3] - self.ref_origin

        x = float(np.dot(rel, self.ref_x))
        y = float(np.dot(rel, self.ref_y))

        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0

        self.robot_pub.publish(msg)

        self.get_logger().info(
            f"Robot(ID 3): x={x:.2f} m , y={y:.2f} m"
        )

    # ----------------------------------------------------

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.ref_origin is None:
            self.get_logger().warn("Reference frame not ready")
            return

        # Back-project pixel onto ground plane
        pixel = np.array([[[x, y]]], dtype=np.float32)
        ray = cv2.undistortPoints(
            pixel,
            self.camera_matrix,
            self.dist_coeffs
        )[0][0]

        scale = self.ref_origin[2] / ray[1]
        cam_point = np.array([
            ray[0] * scale,
            ray[1] * scale,
            self.ref_origin[2]
        ])

        rel = cam_point - self.ref_origin

        gx = float(np.dot(rel, self.ref_x))
        gy = float(np.dot(rel, self.ref_y))

        goal = Point()
        goal.x = gx
        goal.y = gy
        goal.z = 0.0

        self.goal_pub.publish(goal)

        self.get_logger().info(
            f"Clicked Goal: x={gx:.2f} m , y={gy:.2f} m"
        )

        cv2.circle(self.latest_frame, (x, y), 6, (0, 0, 255), -1)
        cv2.imshow(self.window, self.latest_frame)

# --------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ReferenceClickLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
