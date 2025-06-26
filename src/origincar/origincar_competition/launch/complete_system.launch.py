#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    # 声明参数
    launch_args = [
        # AI系统参数
        DeclareLaunchArgument('line_following_speed', default_value='1.15',
                              description='Speed for line following (m/s)'),
        DeclareLaunchArgument('line_kp', default_value='0.003225',
                              description='Proportional gain for line following'),
        DeclareLaunchArgument('cone_avoidance_speed', default_value='0.4',
                              description='Base speed during cone avoidance (m/s)'),
        DeclareLaunchArgument('cone_detection_y_threshold', default_value='130.0',
                              description='Cone height (px) to trigger avoidance action'),
        DeclareLaunchArgument('cone_critical_y_threshold', default_value='250.0',
                              description='Cone height (px) for critical avoidance'),
        DeclareLaunchArgument('cone_avoidance_steering_gain', default_value='0.5',
                              description='Base steering gain for cone avoidance'),
        DeclareLaunchArgument('cone_lateral_offset_threshold', default_value='30.0',
                              description='Cone lateral offset tolerance for centered critical check (px)'),
        DeclareLaunchArgument('post_avoidance_forward_search_duration', default_value='0.5',
                              description='Duration to search forward after cone avoidance (seconds)'),
        DeclareLaunchArgument('post_avoidance_recovery_turn_duration', default_value='0.0',
                              description='Duration for recovery turn (0.0 = infinite)'),
        DeclareLaunchArgument('recovery_turn_linear_speed_ratio', default_value='0.35',
                              description='Linear speed ratio during recovery turn'),
        DeclareLaunchArgument('search_swing_frequency', default_value='0.3',
                              description='Frequency for swing search (Hz)'),
        
        # 优先级控制器参数
        DeclareLaunchArgument('ai_priority', default_value='1',
                              description='AI system priority (1=high, 0=low)'),
        DeclareLaunchArgument('cv_priority', default_value='0',
                              description='CV system priority (1=high, 0=low)'),
        DeclareLaunchArgument('timeout_ai', default_value='0.5',
                              description='AI system timeout (seconds)'),
        DeclareLaunchArgument('timeout_cv', default_value='0.3',
                              description='CV system timeout (seconds)'),
        DeclareLaunchArgument('emergency_stop_timeout', default_value='1.0',
                              description='Emergency stop timeout (seconds)'),
    ]

    # 包含原有的启动文件
    pkg_share = get_package_share_directory('origincar_competition')
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, '/launch/start.launch.py'])
    )

    # QR码识别节点
    qr_node = Node(
        package="qr_decoder",
        executable="qr_decoder",
        name="qr_decoder_node",
        output='screen'
    )

    # AI竞争控制节点
    competition_node = Node(
        package="origincar_competition",
        executable="origincar_competition",
        name="complete_control_node",
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{
            'line_following_speed': LaunchConfiguration('line_following_speed'),
            'line_kp': LaunchConfiguration('line_kp'),
            'cone_avoidance_speed': LaunchConfiguration('cone_avoidance_speed'),
            'cone_detection_y_threshold': LaunchConfiguration('cone_detection_y_threshold'),
            'cone_critical_y_threshold': LaunchConfiguration('cone_critical_y_threshold'),
            'cone_avoidance_steering_gain': LaunchConfiguration('cone_avoidance_steering_gain'),
            'cone_lateral_offset_threshold': LaunchConfiguration('cone_lateral_offset_threshold'),
            'post_avoidance_forward_search_duration': LaunchConfiguration('post_avoidance_forward_search_duration'),
            'post_avoidance_recovery_turn_duration': LaunchConfiguration('post_avoidance_recovery_turn_duration'),
            'recovery_turn_linear_speed_ratio': LaunchConfiguration('recovery_turn_linear_speed_ratio'),
            'search_swing_frequency': LaunchConfiguration('search_swing_frequency'),
        }]
    )

    # CV巡线节点
    line_follower_node = Node(
        package="line_follower",
        executable="line_follower",
        name="line_follower_node",
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 优先级控制器节点
    priority_controller_node = Node(
        package="origincar_competition",
        executable="priority_controller",
        name="priority_controller_node",
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{
            'ai_priority': LaunchConfiguration('ai_priority'),
            'cv_priority': LaunchConfiguration('cv_priority'),
            'timeout_ai': LaunchConfiguration('timeout_ai'),
            'timeout_cv': LaunchConfiguration('timeout_cv'),
            'emergency_stop_timeout': LaunchConfiguration('emergency_stop_timeout'),
        }]
    )

    # 返回启动描述
    return LaunchDescription(launch_args + [
        included_launch,
        qr_node,
        competition_node,
        line_follower_node,
        priority_controller_node
    ]) 