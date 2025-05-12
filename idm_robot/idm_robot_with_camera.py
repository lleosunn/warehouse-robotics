""" 
Authors: Richard Chen, rachen@mit.edu; James Shaw, jshaw4@mit.edu; Eve Silfanus, eves@mit.edu

Much credit to this code is from the course 1.041: Transportation Foundations and Methods in Spring 2024.
"""

# import packages
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import math
import threading
import numpy as np
from scipy import stats

cMat = np.array([[4247.34391, 0.0, 1872.32624], [0.0, 4244.03498, 888.057648], [0.0, 0.0, 1.0]])
dcoeff = np.array([-0.5073313, 0.41420746, 0.00385624, 0.00358224, -1.69381974])

# IDM related parameters
# TO_DO Define parameter specifications of your IDM model and the IDM variant

T = 1 # desired time headway
a = 0.2 # comfortable acceleration (m/s^2)
b = 0.14 # comfortable deceleration (m/s^2)
s_0 = 0.5 # minimum spacing
v_0 = 0.5 # max velocity (m/s)
delta = 4 # exponent

# most recent camera measurements
recent_depths = []
recent_times = []
list_limit = 100 # hold 100 most recent measurements

def compute_gap(depth_list, last_messages=list_limit):
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

def compute_delta_v(time_list, depth_list, last_messages=list_limit):
    """
    Compute delta_v, differential in velocity

    Parameters
    ----------
        time_list: list
            List of recent time measurements.
        depth_list: list
            List of recent depth measurements.
        last_messages: int
            How many of the last few messages to take a look at
    """

    slope, _, _, _, _ = stats.linregress(time_list[-last_messages:], depth_list[-last_messages:])

    return slope

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
    current_gap = compute_gap(recent_depths)
    delta_v = compute_delta_v(recent_times, recent_depths)


    # TO_DO : Write the IDM model below
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

        self.pub2 = self.create_publisher(Twist, 'robomaster_2/cmd_vel', 10) # 10 is the queue size

        self.cam_sub = self.create_subscription(
                Image,
                '/robomaster_2/camera_0/image_raw',
                self.depth_callback,
                qos_profile_sensor_data
                )

    def depth_callback(self, msg):
        global recent_depths
        global recent_times

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_100)
            dt = cv2.aruco.DetectorParameters_create()

            corners, ids, _ = cv2.aruco.detectMarkers(depth_image, dict, parameters=dt)

            if ids is not None and ids.size > 0:
                _, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.05, cMat, dcoeff)
                dist = np.linalg.norm(tvec)
                if len(recent_depths) < list_limit:
                    recent_depths.append(dist)
                    recent_times.append(time.time())
                else: # reset the recent depths and times
                    recent_depths = []
                    recent_times = []

                # print(f"Distance: {dist:.2f} cm")

            else:
                print("No aruco marker")

        except Exception as e:
            self.get_logger().error(f"Could not convert depth image: {e}")
            return


    def run(self):
        global robot_1_pos
        global robot_2_pos
        global robot_1_vel
        global robot_2_vel
        global robot_1_accel
        global robot_2_accel

        last_time = time.time()


        # run IDM model
        # while time.time() - start_time < total_time:
        #     # delta time tracking
        
        if len(recent_times) == list_limit:
            new_time = time.time()
            frame_time = new_time - last_time
            last_time = new_time
            print(f"Frame time: {frame_time}")
            # send command to robots to move

            twist_robot2 = Twist()
            twist_robot2.linear.x = robot_2_vel
            twist_robot2.angular.z = 0.024*robot_2_vel # add correction for slight left turn

            self.pub2.publish(twist_robot2)


            # exit out once done
            # if idx >= float(total_time_steps):
            #     break


            # if start_time + 0.1*idx < time.time():

            # compute acceleration

            robot_2_accel = IDM_model()

            # update position and velocity
            robot_2_pos = robot_2_pos + robot_2_vel*frame_time
            robot_2_vel = np.clip(robot_2_vel + robot_2_accel*frame_time, 0, v_0)

        # stop the robots
        twist_robot2 = Twist()
        twist_robot2.linear.x = 0.0
        
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
