from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='car_follower',
			executable='depth_follower_node',
			name='depth_follower_node',
			output='screen'
		)
	])
