from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('ws_visualize'),
        'urdf', 'husky_dual_fr3.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    
    rviz_config_file = os.path.join(
        get_package_share_directory("ws_visualize"),
        "rviz", "husky_dual_fr3_rviz.rviz")

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    ee_sampler = Node(
        package='ws_visualize',
        executable='ee_sampler_node',
        name='ee_Sampler_node',
        output='screen',
        parameters = [{
            'urdf_path'  : urdf_path
        }]
    )

    return LaunchDescription([
        robot_state_publisher,
        # joint_state_publisher,
        ee_sampler,
        rviz_node
    ])