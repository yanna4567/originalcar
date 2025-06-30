import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # --- 声明 Competition Node (CompleteControl) 的参数 ---
    launch_args = [
        DeclareLaunchArgument('line_following_speed', default_value='0.65',
                              description='Speed for line following (m/s)'),
        DeclareLaunchArgument('line_kp', default_value='0.005225', # 1.0/320.0 0.003225
                              description='Proportional gain for line following'),
        DeclareLaunchArgument('cone_avoidance_speed', default_value='0.4',
                              description='Base speed during cone avoidance (m/s)'),
        DeclareLaunchArgument('cone_detection_y_threshold', default_value='130.0',
                              description='Cone height (px) to trigger avoidance action'),
        DeclareLaunchArgument('cone_critical_y_threshold', default_value='250.0',
                              description='Cone height (px) for critical avoidance'),
        DeclareLaunchArgument('cone_avoidance_steering_gain', default_value='0.65', # 避障k值 0.5
                              description='Base steering gain for cone avoidance and recovery turn amplitude (rad/s)'),
        DeclareLaunchArgument('cone_lateral_offset_threshold', default_value='40.0',  #30
                              description='Cone lateral offset tolerance for centered critical check (px)'),
        DeclareLaunchArgument('post_avoidance_forward_search_duration', default_value='0.5',#避障后向前短时直行
                              description='Duration to search forward after cone avoidance (seconds)'),
        DeclareLaunchArgument('post_avoidance_recovery_turn_duration', default_value='0.0', # 0.0 表示无限恢复转向
                              description='Max duration for recovery turn search (seconds, 0 for indefinite)'),
        DeclareLaunchArgument('recovery_turn_linear_speed_ratio', default_value='0.3',
                              description='Linear speed ratio for recovery turn (relative to line_following_speed)'),
        DeclareLaunchArgument('search_swing_frequency', default_value='0.3', # 仍然用于当last_avoidance_turn_direction_为NONE时的S型摆动
                              description='Frequency for S-swing search if no specific recovery direction (Hz)'),

        # 如果 usb_websocket_display.launch.py 需要参数:
        # DeclareLaunchArgument('video_device', default_value='/dev/video0', description='USB camera device'),
    ]

    # --- 包含 usb_websocket_display.launch.py ---
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('origincar_bringup'), 'launch', 'usb_websocket_display.launch.py')
        ]),
        # launch_arguments={ # 如果需要传递参数
        #     'param_in_included_launch': LaunchConfiguration('param_declared_above_for_main_launch'),
        # }.items()
    )

    # --- QrCodeDetection 节点 ---
    qr_node = Node(
        package="qr_decoder",
        executable="qr_decoder",
        name="qr_code_detection_node",
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # --- Competition (CompleteControl) 节点 ---
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

    # --- 返回 LaunchDescription ---
    return LaunchDescription(launch_args + [
        # included_launch,
        qr_node,
        competition_node
    ])