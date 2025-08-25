#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Package directories
    pkg_robot_description = get_package_share_directory('robot_description')
    
    # Launch arguments
    use_jsp_arg = DeclareLaunchArgument(
        'use_jsp',
        default_value='false',
        description='Whether to launch joint_state_publisher (disabled for Gazebo)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 1. 로봇 모델(URDF) 로드
    urdf_file = os.path.join(pkg_robot_description, 'urdf', 'robot.urdf.xacro') 
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description_param = {'robot_description': robot_description_content}

    # 2. robot_state_publisher 노드 추가
    # URDF를 기반으로 로봇의 각 링크(link) 사이의 TF를 발행합니다.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 3. joint_state_publisher 노드 추가 (조건부 실행)
    # Gazebo가 joint_states를 발행하므로 시뮬레이션 환경에서는 불필요
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('use_jsp')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch Gazebo with robot
    #gazebo_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(pkg_robot_description, 'launch', 'gazebo.launch.py')
    #    )
    #)
    
    # Launch obstacle detector
    obstacle_detector_node = Node(
        package='obstacle_detection',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen'
    )
    
    # Launch A* planner
    astar_planner_node = Node(
        package='path_planner',
        executable='astar_planner',
        name='astar_planner',
        output='screen'
    )
    
    # Launch MPC controller
    #mpc_controller_node = Node(
    #    package='mpc_controller',
    #    executable='acados_mpc_node',
    #    name='acados_mpc_controller',
    #    output='screen'
    #)
    
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
        use_jsp_arg,
        use_sim_time_arg,
        
        # robot_description_launch는 제거하고 아래 두 노드를 추가
        robot_state_publisher_node,
        joint_state_publisher_node,
        
        #gazebo_launch,
        obstacle_detector_node,
        astar_planner_node,
        #mpc_controller_node,
        rviz_node
    ])