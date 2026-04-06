import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ArucoArena(Node):

    def __init__(self):
        super().__init__('aruco_click_localization')

        self.sub = self.create_subscription(
            Image,
            '/ip_stream_image',
            self.callback,
            10)

        self.bridge = CvBridge()

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50)

        self.parameters = cv2.aruco.DetectorParameters_create()

        # arena coordinates (meters)
        self.world_points = {
            0: (0.0, 0.0),
            1: (1.5, 0.0),
            2: (0.0, 1.5)
        }

        self.H = None

        cv2.namedWindow("arena")
        cv2.setMouseCallback("arena", self.mouse_click)

        self.current_frame = None

        self.get_logger().info("Click localization node started")


    def callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters)

        if ids is not None:

            ids = ids.flatten()

            pixel_pts = []
            world_pts = []

            for i, marker_id in enumerate(ids):

                if marker_id in self.world_points:

                    c = corners[i][0]
                    center = c.mean(axis=0)

                    pixel_pts.append(center)
                    world_pts.append(self.world_points[marker_id])

                    cv2.circle(frame,
                               (int(center[0]), int(center[1])),
                               6,(0,255,0),-1)

            if len(pixel_pts) >= 3:

                pixel_pts = np.array(pixel_pts, dtype=np.float32)
                world_pts = np.array(world_pts, dtype=np.float32)

                self.H, _ = cv2.findHomography(pixel_pts, world_pts)

        self.current_frame = frame

        cv2.imshow("arena", frame)
        cv2.waitKey(1)


    def mouse_click(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN and self.H is not None:

            pixel = np.array([x, y, 1.0])
            world = self.H @ pixel

            world = world / world[2]

            X = world[0]
            Y = world[1]

            print(f"\nClicked Arena Position -> X: {X:.2f} m , Y: {Y:.2f} m")


def main(args=None):

    rclpy.init(args=args)

    node = ArucoArena()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
