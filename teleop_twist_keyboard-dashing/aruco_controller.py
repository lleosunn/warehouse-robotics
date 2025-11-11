#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import math

class ArucoControllerNode(Node):
    def __init__(self):
        super().__init__('aruco_controller_node')

        # --- Declare parameters (editable via command line or parameter file) ---
        # Target position in camera frame (meters)
        self.declare_parameter('target_x', -1.1)  # Target x position
        self.declare_parameter('target_y', -0.5)  # Target y position
        
        # P controller gains
        self.declare_parameter('kp_x', 1.0)  # Proportional gain for x
        self.declare_parameter('kp_y', 2.0)  # Proportional gain for y
        self.declare_parameter('kp_yaw', 1.0)  # Proportional gain for yaw
        
        # Maximum velocities (m/s)
        self.declare_parameter('max_linear_x', 0.5)
        self.declare_parameter('max_linear_y', 0.5)
        self.declare_parameter('max_angular_z', 1.0)  # Maximum angular velocity (rad/s)
        
        # Position tolerance (meters) - stop when within this distance
        self.declare_parameter('position_tolerance', 0.05)
        
        # Control rate (Hz)
        self.declare_parameter('control_rate', 20.0)

        # --- Subscribers ---
        self.pose_sub = self.create_subscription(
            Pose,
            '/aruco_pose',
            self.pose_callback,
            10
        )

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/robomaster_1/cmd_vel',
            10
        )

        # Current pose (will be updated by subscription)
        self.current_pose = None
        self.last_pose_time = None

        # --- Control timer ---
        control_period = 1.0 / self.get_parameter('control_rate').value
        self.control_timer = self.create_timer(control_period, self.control_callback)

        self.get_logger().info("📡 ArUco Controller Node started")
        self.get_logger().info(f"  Target position: x={self.get_parameter('target_x').value:.3f}, y={self.get_parameter('target_y').value:.3f}")
        self.get_logger().info(f"  P gains: kp_x={self.get_parameter('kp_x').value:.3f}, kp_y={self.get_parameter('kp_y').value:.3f}")

    def pose_callback(self, msg):
        """Update current pose from ArUco detection."""
        self.current_pose = msg
        self.last_pose_time = self.get_clock().now()

    def control_callback(self):
        """Main control loop - calculates and publishes velocity commands."""
        # Check if we have a valid pose
        if self.current_pose is None:
            # No pose received yet, stop the robot
            self.publish_stop()
            return

        # Check if pose is stale (older than 1 second)
        if self.last_pose_time is not None:
            time_since_pose = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
            if time_since_pose > 1.0:
                self.get_logger().warn(f"⚠️ No pose update for {time_since_pose:.2f}s, stopping")
                self.publish_stop()
                return

        # Get current position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # Extract yaw from quaternion (simple extraction)
        qx = self.current_pose.orientation.x
        qy = self.current_pose.orientation.y
        qz = self.current_pose.orientation.z
        qw = self.current_pose.orientation.w
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Get target position and gains from parameters
        target_x = self.get_parameter('target_x').value
        target_y = self.get_parameter('target_y').value
        target_yaw = 0.0  # Always target 0 yaw
        kp_x = self.get_parameter('kp_x').value
        kp_y = self.get_parameter('kp_y').value
        kp_yaw = self.get_parameter('kp_yaw').value
        max_linear_x = self.get_parameter('max_linear_x').value
        max_linear_y = self.get_parameter('max_linear_y').value
        max_angular_z = self.get_parameter('max_angular_z').value
        tolerance = self.get_parameter('position_tolerance').value

        # Calculate position errors
        error_x = target_x - current_x
        error_y = target_y - current_y
        error_distance = (error_x**2 + error_y**2)**0.5
        
        # Calculate yaw error (normalize to [-pi, pi])
        error_yaw = target_yaw - current_yaw
        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi

        # Check if we're close enough to target
        if error_distance < tolerance:
            # Within tolerance, stop
            self.publish_stop()
            self.get_logger().info(f"✓ At target position (error: {error_distance:.3f}m)")
            return

        # P controller: velocity = Kp * error
        vel_x = kp_x * error_x
        vel_y = kp_y * error_y
        vel_yaw = kp_yaw * error_yaw

        # Limit velocities
        vel_x = max(-max_linear_x, min(max_linear_x, vel_x))
        vel_y = max(-max_linear_y, min(max_linear_y, vel_y))
        vel_yaw = max(-max_angular_z, min(max_angular_z, vel_yaw))

        # Create and publish twist message
        twist = Twist()
        twist.linear.x = float(vel_x)
        twist.linear.y = float(vel_y)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(vel_yaw)

        self.cmd_vel_pub.publish(twist)

        # Log control info
        self.get_logger().info(
            f"Current: ({current_x:.3f}, {current_y:.3f}, {math.degrees(current_yaw):.1f}°) | "
            f"Target: ({target_x:.3f}, {target_y:.3f}, 0.0°) | "
            f"Error: ({error_x:.3f}, {error_y:.3f}, {math.degrees(error_yaw):.1f}°) | "
            f"Vel: ({vel_x:.3f}, {vel_y:.3f}, {math.degrees(vel_yaw):.1f}°/s)"
        )

    def publish_stop(self):
        """Publish zero velocity to stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

