import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    device_used = LaunchConfiguration('device_used')

    device_used_launch_arg = DeclareLaunchArgument(
            'device_used',
            default_value=TextSubstitution(text='webcam'),
            description='Device used for image processing'
        )
    
    thinklet_img_pub_node = Node(
            package='sign_agent',
            executable='img_pub',
            name='thinklet_image_publisher',
            output='screen',
            parameters=[{'device_used': device_used}]
        )
    
    # webcam_img_pub_node = Node(
    #         package='sign_agent',
    #         executable='img_pub_web',
    #         name='webcam_image_publisher',
    #         output='screen',
    #         condition=LaunchConfigurationNotEquals('device_used', 'Thinklet')
    #     )
    
    image_processor_node = Node(
            package='sign_agent',
            executable='img_proc',
            name='image_processor',
            parameters=[{'device_used': device_used}]
        )
    
    machine_state_node = Node(
            package='sign_agent',
            executable='machine_state',
            name='machine_state',
            output='screen'
        )
    
    return LaunchDescription([
        device_used_launch_arg,
        thinklet_img_pub_node,
        # webcam_img_pub_node,
        image_processor_node,
        machine_state_node
    ])