import rclpy
from rclpy.node import Node

import cv2

from geometry_msgs.msg import Point


class GoalClick(Node):

    def __init__(self):

        super().__init__('goal_click_node')

        self.pub = self.create_publisher(
            Point,
            '/goal_point',
            10
        )

        cv2.namedWindow("arena")

        cv2.setMouseCallback("arena",self.click)

        self.get_logger().info("Click goal node started")

    def click(self,event,x,y,flags,param):

        if event==cv2.EVENT_LBUTTONDOWN:

            goal = Point()

            goal.x = float(x)
            goal.y = float(y)

            self.pub.publish(goal)

            print(f"Goal clicked: {goal.x},{goal.y}")


def main(args=None):

    rclpy.init(args=args)

    node = GoalClick()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()
