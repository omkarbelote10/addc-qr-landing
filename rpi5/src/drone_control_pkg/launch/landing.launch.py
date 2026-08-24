from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    target_lat = LaunchConfiguration("target_lat")
    target_lon = LaunchConfiguration("target_lon")

    target_lat_arg = DeclareLaunchArgument('target_lat', default_value='-35.3632171', description="Target Latitude")
    target_lon_arg = DeclareLaunchArgument('target_lon', default_value='149.1652704', description="Target Longitude")



    mission_control_node = Node(
        package='drone_control_pkg',
        executable='mission_control_node',
        name='mission_control_node',
        parameters=[
            {
                'target_lat': target_lat,
                'target_lon': target_lon
            }
        ]
    )

    vision_tracker_node = Node(
        package = 'drone_control_pkg',
        executable='vision_tracker_node',
        name = 'vision_tracker_node'

    )

    return LaunchDescription([
        target_lat_arg,
        target_lon_arg,
        mission_control_node,
        vision_tracker_node
    ])