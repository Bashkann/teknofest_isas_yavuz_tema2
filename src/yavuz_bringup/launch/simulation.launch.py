#!/usr/bin/env python3
"""
YAVUZ AUV - Ana Simülasyon Launch Dosyası
Gazebo Harmonic + ROS2 Jazzy
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ——— Paket dizinleri ———
    yavuz_sim  = get_package_share_directory('yavuz_simulation')
    yavuz_desc = get_package_share_directory('yavuz_description')
    yavuz_nav  = get_package_share_directory('yavuz_navigation')

    # ——— Launch Argümanları ———
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(yavuz_sim, 'worlds', 'teknofest_theme2.sdf'),
        description='Gazebo dunya dosyasi'
    )
    gz_gui_arg = DeclareLaunchArgument(
        'gz_gui', default_value='true',
        description='Gazebo GUI baslat'
    )
    spawn_x_arg       = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg       = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg       = DeclareLaunchArgument('spawn_z', default_value='-0.5')
    start_mission_arg = DeclareLaunchArgument('start_mission', default_value='false')
    buoy_x_arg        = DeclareLaunchArgument('buoy_x',  default_value='25.0')
    buoy_y_arg        = DeclareLaunchArgument('buoy_y',  default_value='-10.0')
    end_x_arg         = DeclareLaunchArgument('end_x',   default_value='45.0')
    end_y_arg         = DeclareLaunchArgument('end_y',   default_value='5.0')
    use_gt_arg        = DeclareLaunchArgument('use_ground_truth', default_value='true')

    # ——— Yapılandırma değişkenleri ———
    world         = LaunchConfiguration('world')
    gz_gui        = LaunchConfiguration('gz_gui')
    spawn_x       = LaunchConfiguration('spawn_x')
    spawn_y       = LaunchConfiguration('spawn_y')
    spawn_z       = LaunchConfiguration('spawn_z')
    start_mission = LaunchConfiguration('start_mission')
    use_gt        = LaunchConfiguration('use_ground_truth')

    # ——— URDF / Xacro ———
    urdf_file = os.path.join(yavuz_desc, 'urdf', 'yavuz_auv.urdf.xacro')
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str
    )

    # ══════════════════════════════════════════════════════════
    # 1. GAZEBO HARMONIC
    # ══════════════════════════════════════════════════════════
    gz_sim_gui = ExecuteProcess(
        cmd=['gz', 'sim', world],
        additional_env={
            'GZ_SIM_RESOURCE_PATH': os.path.join(yavuz_sim, 'models'),
        },
        output='screen',
        condition=IfCondition(gz_gui)
    )

    gz_sim_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', world],
        output='screen',
        condition=UnlessCondition(gz_gui)
    )

    # ══════════════════════════════════════════════════════════
    # 2. ROBOT STATE PUBLISHER
    # ══════════════════════════════════════════════════════════
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ══════════════════════════════════════════════════════════
    # 3. ARAÇ SPAWN
    # ══════════════════════════════════════════════════════════
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_yavuz',
                arguments=[
                    '-name', 'yavuz_auv',
                    '-topic', 'robot_description',
                    '-x', spawn_x,
                    '-y', spawn_y,
                    '-z', spawn_z,
                    '-Y', '0.0',
                ],
                output='screen'
            )
        ]
    )

    # ══════════════════════════════════════════════════════════
    # 4. ROS-GZ KÖPRÜSÜ
    # ══════════════════════════════════════════════════════════
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/yavuz/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/yavuz/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/yavuz/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/yavuz/thruster0/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/yavuz/thruster1/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/yavuz/thruster2/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/yavuz/thruster3/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/yavuz/thruster4/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/yavuz/thruster5/cmd@std_msgs/msg/Float64]gz.msgs.Double',
        ]
    )

    # ══════════════════════════════════════════════════════════
    # 5. EKF LOKALİZASYON
    # ══════════════════════════════════════════════════════════
    ekf_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='yavuz_localization',
            executable='pose_estimator',
            name='yavuz_ekf_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'use_ground_truth': use_gt,
                'publish_tf': True,
            }]
        )]
    )

    # ══════════════════════════════════════════════════════════
    # 6. GÖREV DÜĞÜMÜ
    # ══════════════════════════════════════════════════════════
    mission_node = TimerAction(
        period=7.0,
        actions=[Node(
            package='yavuz_navigation',
            executable='mission_node',
            name='yavuz_mission_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'buoy_x': LaunchConfiguration('buoy_x'),
                'buoy_y': LaunchConfiguration('buoy_y'),
                'end_x':  LaunchConfiguration('end_x'),
                'end_y':  LaunchConfiguration('end_y'),
                'auto_start': start_mission,
                'use_ground_truth': use_gt,
            }]
        )]
    )

    # ══════════════════════════════════════════════════════════
    # 7. RVIZ2
    # ══════════════════════════════════════════════════════════
    rviz_config = os.path.join(yavuz_nav, 'config', 'yavuz_rviz.rviz')
    rviz_node = TimerAction(
        period=6.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            parameters=[{'use_sim_time': True}]
        )]
    )

    # ══════════════════════════════════════════════════════════
    # LAUNCH DESCRIPTION
    # ══════════════════════════════════════════════════════════
    return LaunchDescription([
        # Argümanlar
        world_arg, gz_gui_arg,
        spawn_x_arg, spawn_y_arg, spawn_z_arg,
        start_mission_arg, use_gt_arg,
        buoy_x_arg, buoy_y_arg, end_x_arg, end_y_arg,

        # Bilgi
        LogInfo(msg="\n" + "="*50 +
               "\n YAVUZ AUV Simulasyonu Baslatiliyor..." +
               "\n TEKNOFEST 2026 - Ileri Kategori Tema 2" +
               "\n" + "="*50),

        # Dugumler
        robot_state_pub,
        gz_sim_gui,
        gz_sim_headless,
        spawn_entity,
        ros_gz_bridge,
        ekf_node,
        mission_node,
        rviz_node,

        LogInfo(msg="Gorev baslatmak icin:\n"
               "  ros2 topic pub /yavuz/mission/command "
               "std_msgs/msg/String '{data: start}' --once"),
    ])
