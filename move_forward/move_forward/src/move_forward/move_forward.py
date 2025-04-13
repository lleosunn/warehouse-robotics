import rclpy
from rclpy.node import Node
import sys
from robomaster_msgs.msg import WheelSpeed
import time

class MoveForward(Node):
	def __init__(self):
		super().__init__('move_forward')
		self.pub = self.create_publisher(WheelSpeed, 'robomaster_1/cmd_wheels',10)
	
	def move_for_distance(self, distance_meters, speed_value = 50):
    		# calculates rough time for the robot to move based on distance and speed. This is for testing purposes and will be replaced with emergency stop or other mechanism.
    		approx_speed_m_per_s = 1.0
    		travel_time = distance_meters / approx_speed_m_per_s
    		
    		#message to set all wheels to the given speed_value:
    		msg = WheelSpeed()
    		print(msg)
    		msg.fr = speed_value
    		msg.fl = speed_value
    		msg.rr = speed_value
    		msg.rl = speed_value
    		
    		self.get_logger().info("Starting forward motion for ~{travel_time.1f} seconds")
    		
    		start_time = time.time()
    		while (time.time() - start_time) <travel_time:
    			self.pub.publish(msg)
    			time.sleep(0.1) # 10 hz
    			
    		# Stop command
    		stop_msg = WheelSpeed()
    		self.pub.publish(stop_msg)
    		self.get_logger().info("Stop moving.")
    		
def main(args=None):
	rclpy.init(args=args)
	node = MoveForward()
	
	node.move_for_distance(distance_meters=10.0, speed_value=50)
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()
