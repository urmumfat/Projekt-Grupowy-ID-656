import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. Start Lidara
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_dir, 'launch', 'sllidar_a1_launch.py')
        )
    )

    # 2. Transformacja: base_link -> laser (zakładam, że lidar pokrywa się ze środkiem robota)
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # 3. Odometria z Lidara (rf2o) - Zastępuje dummy_odom.py!
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,            # Publikuje transformację odom -> base_link
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0                   # Częstotliwość w Hz (dostosuj do obciążenia Jetsona)
        }]
    )

    # 4. SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'odom_frame': 'odom'},
            {'map_frame': 'map'},
            {'base_frame': 'base_link'},
            {'scan_topic': '/scan'},
            {'mode': 'mapping'}
        ]
    )

    """
        # 5. RViz
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/my_robot_code/my_robot_code/slam_config.rviz'],
            output='screen'
        )
    """


    # 6. Automatyczne nagrywanie (rosbag)
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', 'moje_nagranie_slam', '/scan', '/tf', '/tf_static', '/odom', '/map'],
        output='screen'
    )

    # 7. Navigation node CHECK NAMES IN setup.py !!!
    robot_navigation = Node(
        package='code',
        executable='robot_navigation_node',
        name='robot_navigation',
        output='screen'
    )

    # 8. Sending velocities to STM32 CHECK NAMES IN setup.py !!!
    motor_driver = Node(
        package='code',
        executable='motor_driver_node',
        name='motor_driver',
        output='screen'
    )

    return LaunchDescription([
        tf_base_to_laser,
        lidar_launch,
        rf2o_node,
        slam_toolbox,
        #rviz,
        robot_navigation,
        motor_driver,
        record_bag
    ])
