"""
Code for reading camera input.

Authors: James Shaw (jshaw4@mit.edu), Eve Silfanus (eves@mit.edu), Richard Chen (rachen@mit.edu)

Date Created: 05/03/2025, 5:07PM
Date Last Modified: 05/03/2025, 5:34PM
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile

from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.subscription = self.create_subscription(
                Image,
                '/robomaster_2/camera_0/image_raw',
                #self.listener_callback,
                self.depth_callback,
                #qos_profile=self.get_topic_qos('/robomaster_2/camera_0/image_raw')
                qos_profile_sensor_data
                )
        
        self.bridge = CvBridge()


    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f"Could not convert depth image: {e}")
            return

        cv2.imshow('show', depth_image)

        # focus on center patch
        h, w = depth_image.shape
        center_crop = depth_image[h//2 - 20 : h//2 + 20, w//2 - 20:w//2 + 20]

        # estimate distance to object
        est_distance = np.median(center_crop)
        #self.get_logger().info(f"Estimated distance: {est_distance:.2f} m")
    


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    # source: https://github.com/ros-perception/image_pipeline/pull/771/commits/d53f6a98a72fba2e0a36854d5083a3341a2e85a6
    def get_topic_qos(self, topic_name: str) -> QoSProfile:
        """!
        Given a topic name, get the QoS profile with which it is being published
        @param topic_name (str) the topic name
        @return QosProfile the qos profile with which the topic is published. If no publishers exist
        for the given topic, it returns the sensor data QoS. returns None in case ROS1 is being used
        """
        topic_name = self.resolve_topic_name(topic_name)
        topic_info = self.get_publishers_info_by_topic(topic_name=topic_name)

        print(topic_info) # this line is looking good.
        if len(topic_info):
            print(topic_info[0].qos_profile)
            return topic_info[0].qos_profile
        else:
            self.get_logger().warn(f"No publishers available for topic {topic_name}. Using system default QoS for subscriber.")
            return qos_profile_system_default


def main():
    rclpy.init()

    camera_subscriber = CameraSubscriber()

    rclpy.spin(camera_subscriber)

    camera_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
