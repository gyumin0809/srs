import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('erp42_slam')

    # Config file paths
    local_ekf_config_path = os.path.join(pkg_share_dir, 'config', 'local_ekf.yaml')
    global_ekf_config_path = os.path.join(pkg_share_dir, 'config', 'global_ekf.yaml')
    rviz_config_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )

    return LaunchDescription([
        # 1. Sensor Drivers (Velodyne)
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            output='screen',
            parameters=[{'device_ip': '192.168.1.201'}]
        ),
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            output='screen'
        ),
        Node(
            package='velodyne_laserscan',
            executable='velodyne_laserscan_node',
            parameters=[{'ring': 7}],
            remappings=[('velodyne_points', '/velodyne_points'), ('scan', '/scan')],
            output='screen'
        ),

        # 2. TF Publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'velodyne'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0', '0.4', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),

        # 3. Localization (Dual EKF: local and global)
        # Local EKF (odom -> base_link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_local',
            output='screen',
            parameters=[local_ekf_config_path],
            remappings=[('odometry/filtered', 'odometry/filtered_local')]
        ),

        # Global EKF (map -> odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_global',
            output='screen',
            parameters=[global_ekf_config_path],
            remappings=[('odometry/filtered', 'odometry/filtered_global')]
        ),

        # NavSat Transform (GPS to odometry)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'magnetic_declination_radians': 0.125,
                'yaw_offset': 1.5708,
                'publish_filtered_gps': True,
                'wait_for_datum': True,
                'zero_altitude': True,
            }],
            remappings=[
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', 'odometry/filtered_global'), # Use global odom for datum
                ('odometry/gps', 'odometry/gps')
            ]
        ),

        # 4. SLAM
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'use_sim_time': False, # Set to True if using simulation
            }],
            remappings=[('odometry/filtered', 'odometry/filtered_global')]
        ),

        # 5. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])
