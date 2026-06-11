from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='my_bot',
            executable='patrol_node.py',
            name='patrol_node',
            output='screen',
            on_exit=Shutdown(),
        ),

        Node(
            package='my_bot',
            executable='indicator_node.py',
            name='indicator_node',
            output='screen',
            on_exit=Shutdown(),
        ),

        Node(
            package='my_bot',
            executable='environment_speed_node.py',
            name='environment_speed_node',
            output='screen',
            on_exit=Shutdown(),
        ),

        Node(
            package='my_bot',
            executable='mqtt_hmi_bridge.py',
            name='mqtt_hmi_bridge',
            output='screen',
            on_exit=Shutdown(),
            parameters=[{
                'broker_host': '10.1.0.2',
                'broker_port': 1883,
            }],
        ),

    ])
