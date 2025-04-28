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

robot_1_pos = 1 # starting position for robot 1 (m)
robot_1_vel = 0 # starting velocity for robot 1 (m)
robot_2_pos = 0 # starting position for robot 2 (m)
robot_2_vel = 0 # starting velocity for robot 2 (m)

dt = 0.1 # send a message every 0.1 seconds
total_time = 10 # 10 seconds
total_time_steps = int(total_time/dt) # run for 10 seconds

# generate a trajectory for robot 1
robot_1_accel = np.concatenate(np.repeat(0.1, int(5/dt)), np.repeat(-0.1, int(5/dt)))



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

    # This is the net distance between the two vehicles (self and vehicle_lead)
    # (denoted as 's' in the lecture note)
    # use this value (in meters) instead of computing it.
    # current_gap = self.compute_current_lead_gap(vehicle_lead, reference_position_x)
    current_gap = robot_1_pos - robot_2_pos

    # TO_DO : Write the IDM model below
    # v = self.velocity.x
    # delta_v = v - vehicle_lead.velocity.x
    delta_v = robot_2_vel - robot_1_vel
    s_opt = s_0 + max(0, robot_2_vel*T + (robot_2_vel*delta_v)/(2*np.sqrt(a*b)))
    accel = a*(1-np.power((robot_2_vel/v_0),delta) - np.power((s_opt/current_gap),2))

    # TO_DO: You need to set the vehicle's new acceleration here by setting 'self.acceleration' parameter
    # self.acceleration.x = accel

    # TO_DO Modify the 'next_step' method to use euler numerical integration scheme
    # self.next_step(dt, reference_position_x)
    return accel

# def next_step(self, dt, reference_position_x):

#     # TO_DO : In this method, make the velocity update of the vehicle according
#     # to the euler numerical integration scheme.
#     # In updating vehicle the position, we call the method 'update_car_position' with
#     # the amount of change of position (delta change) and reference_position_x as inputs.
#     # Also note that velocity and position are pygame 2-D vectors that can be initiated with v = pygame.Vector2()
#     # More details on pygame vectors can be found here : https://www.pygame.org/docs/ref/math.html
#     velocity_change = pygame.Vector2()

#     # forward euler method
#     velocity_change.xy = self.acceleration.x * dt, 0
#     self.velocity += velocity_change

#     # We make sure that vehicles do not have negative velocities (they do not move backwards)
#     if self.velocity.x < 0:
#         v = pygame.Vector2()
#         v.xy = 0, 0
#         self.velocity = v

#     # we can update the position here. We use backward euler method.
#     # Try to understand the difference between forward euler and backward euler methods.
#     position_change = pygame.Vector2()
#     position_change.xy = self.velocity.x * dt, 0

#     # Calling the method 'update_car_position' with the amount of change of position (position_change) as input
#     # This method call is mandatory to convert position_change to pixel level position_change for rendering purposes
#     self.update_car_position(position_change, reference_position_x)




class IDM_robots(Node):
    """ 
    Class to implement IDM model on two robots.
    """

    def __init__(self):
        """ 
        __init__ method for class.
        """

        super().__init__()

        self.pub1 = self.create_publisher(Twist, 'robomaster_1/cmd_vel', 10) # 10 is the queue size
        self.pub2 = self.create_publisher(Twist, 'robomaster_2/cmd_vel', 10) # 10 is the queue size

    def run(self):
        idx = 0
        start_time = time.time()

        # run IDM model
        while time.time() - start_time < total_time:
            # send command to robots to move
            twist_robot1 = Twist()
            twist_robot1.x = robot_1_vel

            twist_robot2 = Twist()
            twist_robot2.x = robot_2_vel

            self.pub1.publish(twist_robot1)
            self.pub2.publish(twist_robot2)

            # exit out once done
            if idx >= total_time_steps:
                break

            if start_time + 0.1*idx < time.time():
                # compute acceleration
                robot_2_accel = IDM_model()

                # update position and velocity
                robot_1_pos = robot_1_pos + robot_1_vel*dt
                robot_1_vel = np.max(np.min(robot_1_vel + robot_1_accel[idx]*dt, v_0), 0)
                
                robot_2_pos = robot_2_pos + robot_2_vel*dt
                robot_2_vel = np.max(np.min(robot_2_vel + robot_2_accel*dt, v_0), 0)

                idx += 1

        # stop the robots
        twist_robot1 = Twist()
        twist_robot1.x = 0

        twist_robot2 = Twist()
        twist_robot2.x = 0

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