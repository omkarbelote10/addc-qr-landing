from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mission_control_node = Node(
        package='drone_control_pkg',
        executable='mission_control_node',
        name='mission_control_node'
    )

    vision_tracker_node = Node(
        package = 'drone_control_pkg',
        executable='vision_tracker_node',
        name = 'vision_tracker_node'

    )

    return LaunchDescription([
        mission_control_node,
        vision_tracker_node
    ])