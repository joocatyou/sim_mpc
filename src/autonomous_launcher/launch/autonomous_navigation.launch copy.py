#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_robot_description = get_package_share_directory('robot_description')
    
    # Launch robot description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_description, 'launch', 'robot_description.launch.py')
        )
    )
    
    # Launch Gazebo with robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_description, 'launch', 'gazebo.launch.py')
        )
    )
    
    # Launch obstacle detector
    obstacle_detector_node = Node(
        package='obstacle_detection',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('use_jsp'))
    )
    
    # Launch A* planner
    astar_planner_node = Node(
        package='path_planner',
        executable='astar_planner',
        name='astar_planner',
        output='screen'
    )
    
    # Launch MPC controller
    mpc_controller_node = Node(
        package='mpc_controller',
        executable='acados_mpc_node', # Use the new Acados MPC node
        name='acados_mpc_controller', # A more descriptive name
        output='screen'
    )
    
    # Define RViz config file
    rviz_config_file = os.path.join(
        get_package_share_directory('robot_description'), 'rviz', 'autonomous_navigation.rviz'
    )
    
    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    return LaunchDescription([
        robot_description_launch,

        gazebo_launch,
        obstacle_detector_node,
        astar_planner_node,
        mpc_controller_node,
        rviz_node
    ])