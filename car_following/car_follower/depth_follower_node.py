import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2


class CarFollower(Node):
    def __init__(self):
        super().__init__('car_follower')

        # Subscribe to the depth image topic from depth_anything_v2_ros2
        self.subscription = self.create_subscription(
            Image,
            '/depth',  # Make sure this matches your actual published topic
            self.depth_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher to command velocity topic for the following robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/robomaster_2/cmd_vel', 10)
        
        # Convert ROS image -> OpenCV
        self.bridge = CvBridge()

        self.get_logger().info('Car follower node started and waiting for depth images.')

    def depth_callback(self, msg):
        try:
            # Convert depth image to numpy array (float32 expected from Depth Anything)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Could not convert depth image: {e}")
            return

        # Sanity check
        if depth_image is None or depth_image.ndim != 2:
            self.get_logger().warn("Invalid depth image format.")
            return

        # Focus on a center patch (adjust if needed)
        h, w = depth_image.shape
        center_crop = depth_image[h//2 - 20:h//2 + 20, w//2 - 20:w//2 + 20]

        # Clean up NaNs or infs
        center_crop = np.nan_to_num(center_crop, nan=0.0, posinf=0.0, neginf=0.0)

        # Estimate distance to object
        est_distance = np.median(center_crop)
        self.get_logger().info(f"Estimated distance: {est_distance:.2f} m")

        # Control logic: Follow if lead robot is more than 1.5m away
        twist = Twist()
        if est_distance > 2.0:
            twist.linear.x = 0.3
        elif est_distance < 1.0:
            twist.linear.x = 0.0
        else:
            twist.linear.x = 0.1

        # Publish command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CarFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()