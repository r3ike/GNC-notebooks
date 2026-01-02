from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_sim_ros',
            executable='sim_node',
            name='uav_sim'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/odom@nav_msgs/msg/Odometry',
                '/cmd_wrench@geometry_msgs/msg/Wrench'
            ]
        )
    ])
