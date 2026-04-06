import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class ArucoRobotPose(Node):

    def __init__(self):

        super().__init__('aruco_robot_pose')

        self.sub = self.create_subscription(
            Image,
            '/ip_stream_image',
            self.callback,
            10)

        self.pose_pub = self.create_publisher(
            Pose2D,
            '/robot_pose',
            10)

        self.goal_pub = self.create_publisher(
            Point,
            '/goal_point',
            10)

        self.bridge = CvBridge()

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50)

        self.parameters = cv2.aruco.DetectorParameters_create()

        # Arena reference markers
        self.world_points = {
            0: (0.0, 0.0),
            1: (1.5, 0.0),
            2: (0.0, 1.5),
            3: (1.5, 1.5)
        }

        self.robot_id = 10

        self.H = None

        cv2.namedWindow("arena")
        cv2.setMouseCallback("arena", self.mouse_click)

        self.current_frame = None

        self.get_logger().info("Robot pose node started")

    def callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters)

        pixel_pts = []
        world_pts = []

        robot_center = None
        robot_corners = None

        if ids is not None:

            ids = ids.flatten()

            for i, marker_id in enumerate(ids):

                c = corners[i][0]
                center = c.mean(axis=0)

                # reference markers
                if marker_id in self.world_points:

                    pixel_pts.append(center)
                    world_pts.append(self.world_points[marker_id])

                    cv2.circle(frame,
                               (int(center[0]), int(center[1])),
                               6, (0,255,0), -1)

                # robot marker
                if marker_id == self.robot_id:

                    robot_center = center
                    robot_corners = c

        if len(pixel_pts) >= 4:

            pixel_pts = np.array(pixel_pts, dtype=np.float32)
            world_pts = np.array(world_pts, dtype=np.float32)

            self.H, _ = cv2.findHomography(pixel_pts, world_pts)

        # compute robot pose
        if self.H is not None and robot_center is not None:

            p = np.array([robot_center[0], robot_center[1], 1.0])

            world = self.H @ p
            world = world / world[2]

            x = world[0]
            y = world[1]

            # heading
            p0 = robot_corners[0]
            p1 = robot_corners[1]

            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]

            theta = np.arctan2(dy, dx)

            pose = Pose2D()

            pose.x = float(x)
            pose.y = float(y)
            pose.theta = float(theta)

            self.pose_pub.publish(pose)

            cv2.putText(frame,
                        f"Robot: {x:.2f},{y:.2f}",
                        (20,40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,(0,255,0),2)

        self.current_frame = frame

        cv2.imshow("arena", frame)
        cv2.waitKey(1)

    def mouse_click(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN and self.H is not None:

            pixel = np.array([x, y, 1.0])

            world = self.H @ pixel
            world = world / world[2]

            goal = Point()

            goal.x = float(world[0])
            goal.y = float(world[1])

            self.goal_pub.publish(goal)

            print(f"Goal -> X: {goal.x:.2f}, Y: {goal.y:.2f}")


def main(args=None):

    rclpy.init(args=args)

    node = ArucoRobotPose()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
