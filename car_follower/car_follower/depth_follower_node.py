import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CVBridge
import numpy as np
import cv2

class Carfollower(Node):
	def __init__(self):
		super().__init__("car_follower")
		
		#Suvscribe to the delpth image topic
		self.subscription = self.create_subscription(
			Image,
			'/depth', #Check to see if it matches published topic
			self.depth_callback,
			10)
		self.subscription
		
		self.cmd_vel_pub = self.create_publisher(Twist, '/robomaster_2/cmd_vel',  10)
		
		#Convert ROS image --> OpenCV
		self.bridge = CVBridge()
		
		self.get_logger().info("Car follower node started and waiting for depth images.")
		
	def depth_callback(self, msg):
		try:
			depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'passthrough')
		except Exception as e:
			self.get_logger().error("Could not convert depth image: {e}")
			return
		
		# Focus camera to center of leading vehicle
		h, w = depth_imge.shape
		center_crop = depth_image[h//2 - 20:h//2 + 20, w//2 - 20:w//2 + 20)
		
		center_crop = np.nan_to_num(center_crop, nan=0.0, posinf=0.0, neginf=0.0)
		
		# Estimate distance to object
		est_distance = np.median(center_crop)
		self.get_logger().info(f"Estimated distance: {est_distance:.2f} m")
		
		# Control logic: Follow if leading vehicle is more than 1.5m away
		twist = Twist()
		if est_distance > 2.0:
			twist.linear.x = 0.3
		elif est_distance < `.0:
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
	
	if __name__ == '__main__':
		main()
