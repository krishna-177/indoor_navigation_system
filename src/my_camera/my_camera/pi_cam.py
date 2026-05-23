import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import argparse

class CameraNode(Node):
    def __init__(self, ip_arg):
        super().__init__('ip_stream_node')
        
        self.publisher_ = self.create_publisher(Image, 'ip_stream_image', 10)
        
        # 30 FPS is usually enough for IP streams; 0.001 (1000fps) is too high for most CPUs
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        
        # FIX: Construct the URL specifically for your Pi stream
        # Since VLC uses tcp/h264://, we use tcp:// here for OpenCV
        if "://" in ip_arg:
            stream_url = ip_arg.replace("tcp/h264://", "tcp://")
        else:
            stream_url = f"tcp://{ip_arg}"
            
        self.get_logger().info(f'Connecting to stream: {stream_url}')
        
        # Use CAP_FFMPEG to ensure it handles the H.264 stream correctly
        self.cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video capture. Is the Pi streaming?')
            # Do not call rclpy.shutdown() here; it causes the context error you saw.
            # Instead, we just mark the node as inactive.
        
        self.bridge = CvBridge()

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if ret:
            # INTER_LINEAR is usually better for quality; NEAREST is only for speed
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Frame dropped or stream ended.')

def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 IP Camera Streamer')
    parser.add_argument('--ip', type=str, required=True, help='IP address (e.g., 10.42.0.173:8888)')
    cli_args = parser.parse_args()

    rclpy.init(args=args)
    node = CameraNode(cli_args.ip)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        # Proper cleanup sequence
        if node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        # Only shutdown if rclpy is still active
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
