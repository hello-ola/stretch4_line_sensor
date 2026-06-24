from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('stretch4_line_sensor')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'line_sensor.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'line_sensor.rviz'])

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz with the line sensor configuration',
    )

    line_sensor_node = Node(
        package='stretch4_line_sensor',
        executable='line_sensor_node',
        name='line_sensor_node',
        output='screen',
        parameters=[config_file],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        line_sensor_node,
        rviz_node,
    ])
