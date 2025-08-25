import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_robot_description = get_package_share_directory('robot_description')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. Robot Description (URDF)
    urdf_file = os.path.join(pkg_robot_description, 'urdf', 'rtabmap_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    robot_description_param = {'robot_description': robot_description_content}

    # 2. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 3. Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 4. Obstacle Detector
    obstacle_detector_node = Node(
        package='obstacle_detection',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen'
    )
    
    # 5. A* Planner
    astar_planner_node = Node(
        package='path_planner',
        executable='astar_planner',
        name='astar_planner',
        output='screen'
    )
    
    # 6. RViz
    rviz_config_file = os.path.join(
        pkg_robot_description, 'rviz', 'autonomous_navigation.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Odom TF Publisher Node
    odom_tf_publisher_node = Node(
        package='autonomous_launcher',
        executable='odom_tf_publisher',
        name='odom_tf_publisher',
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        odom_tf_publisher_node, # Add the new node here
        obstacle_detector_node,
        astar_planner_node,
        rviz_node
    ])