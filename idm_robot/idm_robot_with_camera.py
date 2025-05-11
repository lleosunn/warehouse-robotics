""" 
Authors: Richard Chen, rachen@mit.edu; James Shaw, jshaw4@mit.edu; Eve Silfanus, eves@mit.edu

Much credit to this code is from the course 1.041: Transportation Foundations and Methods in Spring 2024.
"""

# import packages
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import time
import math
import threading
import numpy as np

# IDM related parameters
# TO_DO Define parameter specifications of your IDM model and the IDM variant

T = 1 # desired time headway
a = 0.2 # comfortable acceleration (m/s^2)
b = 0.14 # comfortable deceleration (m/s^2)
s_0 = 0.5 # minimum spacing
v_0 = 0.5 # max velocity (m/s)
delta = 4 # exponent

robot_1_pos = 1.0 # starting position for robot 1 (m)
robot_1_vel = 0.0 # starting velocity for robot 1 (m)
robot_2_pos = 0.0 # starting position for robot 2 (m)
robot_2_vel = 0.0 # starting velocity for robot 2 (m)

dt = 0.1 # send a message every 0.1 seconds
total_time = 10 # 10 seconds
total_time_steps = int(total_time/dt) # run for 10 seconds

# generate a trajectory for robot 1
robot_1_accel = np.concatenate([np.repeat(0.1, int(5/dt)), np.repeat(-0.1, int(5/dt))])


def compute_gap(depth_list, last_messages=1000):
    """ 
    Compute gap.

    Parameters
    ----------
        depth_list: list
            List of recent depth measurements from camera.
        last_messages: int
            How many of the last few messages to take a look at
    """

    return np.median(depth_list[-last_messages:])

def compute_delta_v(delta_v_list, last_messages=1000):
    """
    Compute delta_v, differential in velocity

    Parameters
    ----------
        delta_v_list: list
            List of recent delta_v measurements.
        last_messages: int
            How many of the last few messages to take a look at
    """

    return np.median(delta_v_list[-last_messages:])


# def IDM_model(self, dt, vehicle_lead, reference_position_x):
def IDM_model():
    """ 
    Implementation of the Intelligent Driver Model (IDM) for car following. Calculates acceleration for the following vehicle.

    /* no longer in use at the moment
    Parameters 
    ----------
        dt: float
            time step
        vehicle_lead: float
            leading vehicle
        reference_position_x: float
            reference position on the roadmap
    */
    """

    # print('here IDM')
    # This is the net distance between the two vehicles (self and vehicle_lead)
    # (denoted as 's' in the lecture note)
    # use this value (in meters) instead of computing it.
    current_gap = robot_1_pos - robot_2_pos

    # TO_DO : Write the IDM model below
    delta_v = robot_2_vel - robot_1_vel
    s_opt = s_0 + max(0, robot_2_vel*T + (robot_2_vel*delta_v)/(2*np.sqrt(a*b)))
    accel = a*(1-np.power((robot_2_vel/v_0),delta) - np.power((s_opt/current_gap),2))

    return accel

class IDM_robots(Node):
    """ 
    Class to implement IDM model on two robots.
    """

    def __init__(self):
        """ 
        __init__ method for class.
        """

        super().__init__('idm_robots')

        self.pub1 = self.create_publisher(Twist, 'robomaster_1/cmd_vel', 10) # 10 is the queue size
        self.pub2 = self.create_publisher(Twist, 'robomaster_2/cmd_vel', 10) # 10 is the queue size

    def run(self):
        global robot_1_pos
        global robot_2_pos
        global robot_1_vel
        global robot_2_vel
        global robot_1_accel
        global robot_2_accel

        idx = 0

        start_time = time.time()
        last_time = time.time();


        # run IDM model
        while time.time() - start_time < total_time:
            # delta time tracking
            new_time = time.time()
            frame_time = new_time - last_time
            last_time = new_time

            # send command to robots to move
            twist_robot1 = Twist()
            twist_robot1.linear.x = robot_1_vel
            twist_robot1.angular.z = 0.029*robot_1_vel # add correction for slight left turn 

            twist_robot2 = Twist()
            twist_robot2.linear.x = robot_2_vel
            twist_robot2.angular.z = 0.024*robot_2_vel # add correction for slight left turn


            self.pub1.publish(twist_robot1)
            self.pub2.publish(twist_robot2)


            # exit out once done
            if idx >= float(total_time_steps):
                break


            if start_time + 0.1*idx < time.time():
                # compute acceleration

                robot_2_accel = IDM_model()

                # update position and velocity
                robot_1_pos = robot_1_pos + robot_1_vel*dt

                robot_1_vel = np.clip(robot_1_vel + robot_1_accel[int(idx)]*dt, 0, v_0)
                

                robot_2_pos = robot_2_pos + robot_2_vel*dt
                robot_2_vel = np.clip(robot_2_vel + robot_2_accel*dt, 0, v_0)

                idx += 1

        # stop the robots
        twist_robot1 = Twist()
        twist_robot1.linear.x = 0.0

        twist_robot2 = Twist()
        twist_robot2.linear.x = 0.0
        
        self.pub1.publish(twist_robot1)
        self.pub2.publish(twist_robot2)


def main(args=None):
    """
    Perform IDM model.
    """

    try:
        rclpy.init(args=args)

        node = IDM_robots()

        spinning = threading.Thread(target=rclpy.spin, args=(node,))
        spinning.start()

        node.run()

        node.destroy_node()


        rclpy.shutdown()
    
    
    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
