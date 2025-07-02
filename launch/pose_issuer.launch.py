from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def generate_launch_descriptions():
    headless = LaunchConfiguration('headless')
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to run in headless mode or not'
    )



    return LaunchDesription([
        Node(
            package='clock_pose_issuer',
            executable='clock_pose_issuer',
            name='clock_pose_issuer',
            output='screen'
        ),
        Node(
            package='target_pose_issuer',
            executable='target_pose_issuer',
            name='target_pose_issuer',
            output='screen'
        ),
        Node(
            package='gui_pose_issuer',
            executable='gui_pose_issuer',
            name='gui_pose_issuer',
            output='screen',
            condition=UnlessCondition(headless)
        )
    ])
