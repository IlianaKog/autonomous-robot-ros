import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('outdoor_robot_bringup')
    description_dir = get_package_share_directory('outdoor_robot_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    world_path = os.path.join(bringup_dir, 'worlds', 'rough_terrain.world')
    urdf_path = os.path.join(description_dir, 'urdf', 'robot.urdf.xacro')
    ekf_config_path = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    twist_mux_config_path = os.path.join(bringup_dir, 'config', 'twist_mux_params.yaml')

    # Start Gazebo Server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    # Start Gazebo Client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', urdf_path])}]
    )

    # Spawn Robot
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'outdoor_robot', '-topic', 'robot_description', '-z', '0.5'],
        output='screen'
    )

    # Node: EKF Local (odom -> base_link)
    start_ekf_local_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='log',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # Node: EKF Global (map -> odom)
    start_ekf_global_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='log',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    # Node: NavSat Transform (GPS -> Odometry)
    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='log',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[ekf_config_path],
        remappings=[('imu/data', 'imu/data'),
                    ('gps/fix', 'gps/fix'), 
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', 'odometry/global')]
    )

    # RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(bringup_dir, 'config', 'navigation.rviz')],
        output='screen'
    )

    # Twist Mux Node
    twist_mux_cmd = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[twist_mux_config_path, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    return LaunchDescription([
        start_gazebo_server_cmd,
        start_gazebo_client_cmd, 
        robot_state_publisher_cmd,
        spawn_robot_cmd,
        start_ekf_local_cmd,
        start_ekf_global_cmd,
        start_navsat_transform_cmd,
        rviz_cmd,
        twist_mux_cmd
    ])
