#!/usr/bin/env python3
"""
YAVUZ AUV - Simülasyon Launch (Düzeltilmiş)
- use_sim_time: false (VM'de zaman senkron sorunu yok)
- pose_bridge dahil (Gazebo pose → lokalizasyon)
- mission_node dahil
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    yavuz_sim  = get_package_share_directory('yavuz_simulation')
    yavuz_desc = get_package_share_directory('yavuz_description')

    # Argümanlar
    gz_gui_arg = DeclareLaunchArgument('gz_gui', default_value='true')
    buoy_x_arg = DeclareLaunchArgument('buoy_x', default_value='25.0')
    buoy_y_arg = DeclareLaunchArgument('buoy_y', default_value='-10.0')
    end_x_arg  = DeclareLaunchArgument('end_x',  default_value='45.0')
    end_y_arg  = DeclareLaunchArgument('end_y',  default_value='5.0')
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='false')

    gz_gui     = LaunchConfiguration('gz_gui')
    world_path = os.path.join(yavuz_sim, 'worlds', 'teknofest_theme2.sdf')
    urdf_file  = os.path.join(yavuz_desc, 'urdf', 'yavuz_auv.urdf.xacro')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str
    )

    # 1. Gazebo
    gz_gui_proc = ExecuteProcess(
        cmd=['gz', 'sim', world_path],
        output='screen',
        condition=IfCondition(gz_gui)
    )
    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', world_path],
        output='screen',
        condition=UnlessCondition(gz_gui)
    )

    # 2. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen'
    )

    # 3. ROS-GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '/yavuz/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/teknofest_theme2/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/yavuz/thruster0/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/yavuz/thruster1/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/yavuz/thruster2/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/yavuz/thruster3/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/yavuz/thruster4/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/yavuz/thruster5/cmd@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )

    # 4. Araç spawn
    spawn = TimerAction(period=3.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'yavuz_auv',
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '-0.3',
            ],
            output='screen'
        )
    ])

    # 5. Pose Bridge (Gazebo → lokalizasyon)
    pose_bridge = TimerAction(period=5.0, actions=[
        Node(
            package='yavuz_navigation',
            executable='pose_bridge',
            name='gazebo_pose_bridge',
            parameters=[{'use_sim_time': False}],
            output='screen'
        )
    ])

    # 6. Mission Node
    mission = TimerAction(period=6.0, actions=[
        Node(
            package='yavuz_navigation',
            executable='mission_node',
            name='yavuz_mission_node',
            parameters=[{
                'use_sim_time': False,
                'buoy_x': LaunchConfiguration('buoy_x'),
                'buoy_y': LaunchConfiguration('buoy_y'),
                'end_x':  LaunchConfiguration('end_x'),
                'end_y':  LaunchConfiguration('end_y'),
                'auto_start': LaunchConfiguration('auto_start'),
                'use_ground_truth': True,
            }],
            output='screen'
        )
    ])

    return LaunchDescription([
        gz_gui_arg, buoy_x_arg, buoy_y_arg,
        end_x_arg, end_y_arg, auto_start_arg,

        LogInfo(msg="\n" + "="*50 +
               "\n YAVUZ AUV - TEKNOFEST 2026 - Tema 2" +
               "\n" + "="*50),

        gz_gui_proc, gz_headless,
        rsp, bridge, spawn,
        pose_bridge, mission,

        LogInfo(msg="\nGorev baslatmak icin:\n"
               "ros2 topic pub /yavuz/mission/command "
               "std_msgs/msg/String '{data: start}' --once"),
    ])
