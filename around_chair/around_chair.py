# Python script to make the RoboMaster go around a chair
# Author: Richard Chen, rachen@mit.edu; Eve Silfanus, eves@mit.edu; James Shaw, jshaw4@mit.edu
# Date created: 04/20/2025, 3:31PM
# Date last modified: 04/20/2025, 3:31PM


# import packages
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import time
import math
import threading


class AroundChair(Node):
    def __init__(self):
        super().__init__('around_chair')

        self.pub = self.create_publisher(Twist, 'robomaster_1/cmd_vel', 10)
        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.cmd_timer = None
        self.stop_timer = None

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        self.pub.publish(msg)

    def forward_step(self, duration=1.0, speed=0.5):
        self.cmd_timer = self.create_timer(0.1, lambda: self.forward(speed=speed))
        self.stop_timer = self.create_timer(duration, self.stop)

    def left_turn(self, duration=1.0, speed=1.0):
        self.cmd_timer = self.create_timer(0.1, lambda: self.left(speed=speed))
        self.stop_timer = self.create_timer(duration, self.stop)

    def right_turn(self, duration=1.0, speed=1.0):
        self.cmd_timer = self.create_timer(0.1, lambda: self.right(speed=speed))
        self.stop_timer = self.create_timer(duration, self.stop)

    def forward(self, speed=0.5):
        msg = Twist() # in meters/second, and radians/second
        msg.linear.x = speed
        # front_length = 1.5 # in meters
        self.pub.publish(msg)
    
    def left(self, speed=1.0):
        msg = Twist()
        msg.angular.z = -speed
        self.pub.publish(msg)

    def right(self, speed=1.0):
        msg = Twist()
        msg.angular.z = speed
        self.pub.publish(msg)

    def stop(self):
        if self.cmd_timer:
            self.cmd_timer.cancel()
        if self.stop_timer:
            self.stop_timer.cancel()

        stop_msg = Twist()
        self.pub.publish(stop_msg)
        

        # i = 0
        # while i < 10:
        #   self.pub.publish(msg)
        #   i+=1
        #   time.sleep(0.1)

        # stop
        # stop_msg = Twist()
        
        # self.pub.publish(stop_msg)


def main(args=None):

    """
    Make robot move forward.
    """
    
    try:
        rclpy.init(args=args) # start working with ROS2
    
        # node = rclpy.create_node('around_chair') # generate node to act on
        node = AroundChair()
        # rclpy.spin(node)

        spinning = threading.Thread(target=rclpy.spin, args=(node,))
        spinning.start()


        curr_duration = 2.5
        node.forward_step(duration=curr_duration)
        time.sleep(curr_duration)


        curr_duration = 1.6 # roughly the number of radians
        node.left_turn(duration=curr_duration)
        time.sleep(curr_duration)

        curr_duration = 2.5
        node.forward_step(duration=curr_duration)
        time.sleep(curr_duration)

        curr_duration = 1.6
        node.left_turn(curr_duration)
        time.sleep(curr_duration)
        
        curr_duration = 2.5
        node.forward_step(duration=curr_duration)
        time.sleep(curr_duration)

        curr_duration = 1.6
        node.left_turn(curr_duration)
        time.sleep(curr_duration)

        curr_duration = 2.5
        node.forward_step(duration=curr_duration)
        time.sleep(curr_duration)

        node.destroy_node()
        
        rclpy.shutdown()

        # TwistMsg = geometry_msgs.msg.Twist

        # pub = node.create_publisher(TwistMsg, 'robomaster_1/cmd_vel', 10) # 

        def timer_callback():
            msg = TwistMsg()
            msg.linear.x = 10.0
            pub.publish(msg)

        # timer = node.create_timer(0.1, timer_callback)

        # try:
         #  rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)

        # finally:
        #  node.destroy_node()
        #  rclpy.shutdown()

    if __name__ == '__main__':
        main()
