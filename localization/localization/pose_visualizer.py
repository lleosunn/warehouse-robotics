#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import time


class PoseVisualizerNode(Node):
    def __init__(self):
        super().__init__('pose_visualizer')

        # Store latest poses and timestamps
        self.camera0_pose = None
        self.camera1_pose = None
        self.camera0_last_time = 0.0
        self.camera1_last_time = 0.0

        # Timeout in seconds - if no message received within this time, consider camera as not seeing marker
        self.pose_timeout = 0.5

        # Subscribers for pose_2 from each camera
        self.camera0_sub = self.create_subscription(
            Pose,
            '/camera0/pose_2',
            self.camera0_callback,
            10
        )
        self.camera1_sub = self.create_subscription(
            Pose,
            '/camera1/pose_2',
            self.camera1_callback,
            10
        )

        # Setup matplotlib figure
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.suptitle('Real-time Pose Visualization', fontsize=14, fontweight='bold')

        # Create scatter plots for each point with different colors
        self.scatter_cam0 = self.ax.scatter([], [], c='#00D4FF', s=150, label='Camera 0', marker='o', edgecolors='white', linewidths=1.5)
        self.scatter_cam1 = self.ax.scatter([], [], c='#FF6B6B', s=150, label='Camera 1', marker='o', edgecolors='white', linewidths=1.5)
        self.scatter_avg = self.ax.scatter([], [], c='#50FA7B', s=200, label='Fused', marker='*', edgecolors='white', linewidths=1.5)

        # Configure axes
        self.ax.set_xlim(-0.5, 1)
        self.ax.set_ylim(-0.5, 1)
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3, linestyle='--')
        self.ax.legend(loc='upper right', fontsize=10)

        # Text annotations for coordinates
        self.text_cam0 = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes, 
                                       fontsize=10, verticalalignment='top', color='#00D4FF',
                                       fontfamily='monospace')
        self.text_cam1 = self.ax.text(0.02, 0.93, '', transform=self.ax.transAxes,
                                       fontsize=10, verticalalignment='top', color='#FF6B6B',
                                       fontfamily='monospace')
        self.text_avg = self.ax.text(0.02, 0.88, '', transform=self.ax.transAxes,
                                      fontsize=10, verticalalignment='top', color='#50FA7B',
                                      fontfamily='monospace')

        # Animation
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)

        self.get_logger().info("📊 Pose Visualizer Node started.")
        self.get_logger().info("   Subscribing to /camera0/pose_2 and /camera1/pose_2")

    def camera0_callback(self, msg):
        self.camera0_pose = (msg.position.x, msg.position.y)
        self.camera0_last_time = time.time()

    def camera1_callback(self, msg):
        self.camera1_pose = (msg.position.x, msg.position.y)
        self.camera1_last_time = time.time()

    def update_plot(self, frame):
        current_time = time.time()

        # Check if camera0 data is still valid (within timeout)
        cam0_valid = (self.camera0_pose is not None and 
                      (current_time - self.camera0_last_time) < self.pose_timeout)
        # Check if camera1 data is still valid (within timeout)
        cam1_valid = (self.camera1_pose is not None and 
                      (current_time - self.camera1_last_time) < self.pose_timeout)

        # Update camera0 scatter
        if cam0_valid:
            self.scatter_cam0.set_offsets([self.camera0_pose])
            self.text_cam0.set_text(f'Cam0: x={self.camera0_pose[0]:.3f}, y={self.camera0_pose[1]:.3f}')
        else:
            self.scatter_cam0.set_offsets(np.empty((0, 2)))
            self.text_cam0.set_text('Cam0: -- (no detection)')

        # Update camera1 scatter
        if cam1_valid:
            self.scatter_cam1.set_offsets([self.camera1_pose])
            self.text_cam1.set_text(f'Cam1: x={self.camera1_pose[0]:.3f}, y={self.camera1_pose[1]:.3f}')
        else:
            self.scatter_cam1.set_offsets(np.empty((0, 2)))
            self.text_cam1.set_text('Cam1: -- (no detection)')

        # Update average/fused scatter - use available data
        if cam0_valid and cam1_valid:
            # Both cameras see the marker - use average
            avg_x = (self.camera0_pose[0] + self.camera1_pose[0]) / 2.0
            avg_y = (self.camera0_pose[1] + self.camera1_pose[1]) / 2.0
            self.scatter_avg.set_offsets([(avg_x, avg_y)])
            self.text_avg.set_text(f'Fused: x={avg_x:.3f}, y={avg_y:.3f} (avg)')
        elif cam0_valid:
            # Only camera0 sees the marker - use its data
            self.scatter_avg.set_offsets([self.camera0_pose])
            self.text_avg.set_text(f'Fused: x={self.camera0_pose[0]:.3f}, y={self.camera0_pose[1]:.3f} (cam0 only)')
        elif cam1_valid:
            # Only camera1 sees the marker - use its data
            self.scatter_avg.set_offsets([self.camera1_pose])
            self.text_avg.set_text(f'Fused: x={self.camera1_pose[0]:.3f}, y={self.camera1_pose[1]:.3f} (cam1 only)')
        else:
            # Neither camera sees the marker
            self.scatter_avg.set_offsets(np.empty((0, 2)))
            self.text_avg.set_text('Fused: -- (no detection)')

        return self.scatter_cam0, self.scatter_cam1, self.scatter_avg

    def run(self):
        """Run the node with matplotlib integration."""
        plt.ion()
        plt.show()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            plt.pause(0.01)

        plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = PoseVisualizerNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
