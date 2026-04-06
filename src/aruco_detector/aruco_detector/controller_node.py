import rclpy
from rclpy.node import Node

import math

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


class Controller(Node):

    def __init__(self):

        super().__init__('controller_node')

        # subscribers
        self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.pose_callback,
            10
        )

        self.create_subscription(
            Point,
            '/goal_point',
            self.goal_callback,
            10
        )

        # publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # robot state
        self.pose = None
        self.goal = None

        # control timer
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info("Controller node started")


    def pose_callback(self,msg):

        self.pose = msg


    def goal_callback(self,msg):

        self.goal = msg

        self.get_logger().info(
            f"New goal: {msg.x:.2f}, {msg.y:.2f}"
        )


    def normalize_angle(self,angle):

        while angle > math.pi:
            angle -= 2 * math.pi

        while angle < -math.pi:
            angle += 2 * math.pi

        return angle


    def control_loop(self):

        if self.pose is None or self.goal is None:
            return


        # -------- parameters --------
        ANGLE_TOL = 0.20      # rad
        DIST_TOL = 0.05       # meters

        MAX_LINEAR = 0.5
        MAX_ANGULAR = 0.6

        LINEAR_GAIN = 1.1
        ANGULAR_GAIN = 0.6
        # -----------------------------


        dx = self.goal.x - self.pose.x
        dy = self.goal.y - self.pose.y

        distance = math.sqrt(dx*dx + dy*dy)

        target_angle = math.atan2(dy,dx)

        angle_error = self.normalize_angle(
            target_angle - self.pose.theta
        )

        cmd = Twist()


        # -------- goal reached --------
        if distance < DIST_TOL:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            return


        # -------- rotate toward goal --------
        if abs(angle_error) > ANGLE_TOL:

            cmd.linear.x = 0.0

            cmd.angular.z = ANGULAR_GAIN * angle_error

            # minimum rotation speed
            if abs(cmd.angular.z) < 0.2:
                cmd.angular.z = 0.2 * math.copysign(1, angle_error)


        # -------- move forward --------
        else:

            cmd.linear.x = LINEAR_GAIN * distance

            cmd.angular.z = 0.0


        # -------- clamp speeds --------
        cmd.linear.x = max(min(cmd.linear.x, MAX_LINEAR), -MAX_LINEAR)

        cmd.angular.z = max(min(cmd.angular.z, MAX_ANGULAR), -MAX_ANGULAR)


        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = Controller()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
