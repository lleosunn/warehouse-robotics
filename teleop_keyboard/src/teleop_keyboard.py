import rclpy
from rclpy.node import Node

import sys
import select
import termios
import tty

from robomaster_msgs.msg import WheelSpeed

def get_key(timeout = 0.1):
	old_settings = termios.tcgetattr(sys.stdin)
	try:
	# cbreak mode
	tty.setcbreak(sys.stdin.fileno())
	
	# wait for input
	rlist, _, _ = select.select([sys.stdin], [], [], timeout)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = None
	finally: 
	# restore terminal settings
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
	return key
	
class KeyboardTeleop(Node):
	def __init__(self):
		super().__init__('keyboard_teleop')
		# publish to wheel-speed topic
		self.pub_wheel = self.create_publisher(WheelSpeed, 'robomaster_1/cmd_wheels', 10)
		
		# define some speed values
		self.forward_speed = 50
		self.turn_speed = 50
		
		self.get_logger().info("KeyboardTeleop node started!")
		self.get_logger().info("Use keys: [W A S D] to move, [Space] stop, [Ctrl - C] to quit")
	def spin(self):
	"""Continously read keys and publish commands"""
		try:
			while rclpy.ok():
				key = get_key()
				if key is None:
					continue
				
				msg = WheelSpeed()
				
				if key.lower() == 'w':
					# move forward
					msg.fr = self.forward_speed 
					msg.fl = self.forward_speed
					msg.rr = self.forward_speed
					msg.rl = self.forward_speed
					self.get_logger().info("Moving Forward")
				if key.lower() == 's':
					# move forward
					msg.fr = -self.forward_speed 
					msg.fl = -self.forward_speed
					msg.rr = -self.forward_speed
					msg.rl = -self.forward_speed
					self.get_logger().info("Moving Backward")
				if key.lower() == 'a':
					# move forward
					msg.fr = self.forward_speed 
					msg.fl = -self.forward_speed
					msg.rr = self.forward_speed
					msg.rl = -self.forward_speed
					self.get_logger().info("Turning Left")
				if key.lower() == 'd':
					# move forward
					msg.fr = -self.forward_speed 
					msg.fl = self.forward_speed
					msg.rr = -self.forward_speed
					msg.rl = self.forward_speed
					self.get_logger().info("Turning Right")
				elif key == ' ':
					self.get_logger().info("Stopped")
				elif ord(key) == 3:
					break
				else: 
					continue
				
				self.pub_wheel.publish(msg)
				
		except KeyboardInterrupt:
			pass
		
		stop_msg = WheelSpeed()
		self.pub_wheel.publish(stop_msg)
		self.get_logger().info("Exiting teleop")

def main(args=None):
	rclpy.init(args=args)
	node = KeyboardTeleop()
	node.spin()
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
