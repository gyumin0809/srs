import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get the share directory of the erp42_slam package
    pkg_share_dir = get_package_share_directory('erp42_slam')

    # Full path to the ekf.yaml file
    ekf_config_path = os.path.join(pkg_share_dir, 'config', 'ekf.yaml')

    # Full path to the rviz config file
    rviz_config_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )

    # Path to the URDF file
    urdf_file_path = os.path.join(pkg_share_dir, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file_path])

    return LaunchDescription([
        # 1. Custom LiDAR Driver
        Node(
            package='erp42_slam',
            executable='lidar_sensor_node',
            output='screen',
            parameters=[{'lidar.remote_ip': '192.168.1.77'}]
        ),

        # 2. Robot State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'publish_frequency': 10.0}]
        ),

        # 3. Robot Localization (GPS + IMU Fusion)
        Node(
            package='robot_localization',
            executable='ekf_node',
            parameters=[ekf_config_path],  # ✅ Use variable
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            parameters=[{
                'frequency': 30.0,
                'magnetic_declination_radians': 0.125,  # ❗ Modify to your local value
                'yaw_offset': 1.5708,                   # ❗ IMU mounting orientation
                'publish_filtered_gps': True,
                'wait_for_datum': True
            }],
            remappings=[
                ('gps/fix', '/gps/fix'),
                ('imu/data', '/imu/data'),
                ('odometry/filtered', '/odometry/filtered')
            ],
            output='screen'
        ),

        # 4. SLAM
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            output='screen',
            remappings=[('/scan', '/lidar/points')],
            parameters=[{'scan_queue_size': 95, 'transform_tolerance': 0.5, 'use_sim_time': False}]
        ),

        # 5. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])
