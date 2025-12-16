import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # ========================================================================
    #                           QUICK CONFIGURATION
    # ========================================================================
    # Edit these variables to change default values without touching complex code.
    
    DEFAULT_TARGET_FREQ = '50.0'   # Expected sensor frequency (Hz)
<<<<<<< HEAD
    DEFAULT_REPORT_FREQ = '10.0'     # UI Table update frequency (Hz) -> 1.0 means once per second
    DEFAULT_RECORD_BAG  = 'false'   # Record bag by default: 'true' or 'false'
=======
    DEFAULT_REPORT_FREQ = '10.0'   # UI Table update frequency (Hz) -> 1.0 means once per second
    DEFAULT_RECORD_BAG  = 'true'   # Record bag by default: 'true' or 'false'
>>>>>>> 6321d4c47f4c9112ad0fbc27cb5ea6dd0c6558c6
    BAG_OUTPUT_DIR      = 'tunel_upo' # Bag folder name if recording is enabled
    
    # Node Configuration
    PACKAGE_NAME = 'eliko_monitor'
    EXECUTABLE   = 'distance_monitor'
    NODE_NAME    = 'distance_monitor_node'
        
    target_freq_arg = DeclareLaunchArgument(
        'target_freq',
        default_value=DEFAULT_TARGET_FREQ,
        description='Expected sensor frequency (Hz) for packet loss calculation.'
    )

    report_freq_arg = DeclareLaunchArgument(
        'report_freq',
        default_value=DEFAULT_REPORT_FREQ,
        description='UI Table update frequency in Hz (e.g., 1.0 for 1s, 5.0 for 200ms).'
    )

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value=DEFAULT_RECORD_BAG,
        description='Set to "true" to automatically record all topics.'
    )

    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value=BAG_OUTPUT_DIR,
        description='Output directory name for the rosbag.'
    )

    # Monitor Node
    monitor_node = Node(
        package=PACKAGE_NAME,
        executable=EXECUTABLE,
        name=NODE_NAME,
        output='screen',
        emulate_tty=True, 
        parameters=[
            {
                'target_frequency': LaunchConfiguration('target_freq'),
                'report_frequency': LaunchConfiguration('report_freq')
            }
        ]
    )

    # Records all topics (-a)
    record_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=['ros2', 'bag', 'record', '-a', '-o', LaunchConfiguration('bag_name')],
        output='screen'
    )

    # Status message
    start_msg = LogInfo(
        msg=[
            'Starting Eliko Monitor... ',
            'Target Freq: ', LaunchConfiguration('target_freq'), ' Hz. ',
            'UI Update Freq: ', LaunchConfiguration('report_freq'), ' Hz. ',
            'Recording enabled: ', LaunchConfiguration('record_bag')
        ]
    )

    
    return LaunchDescription([
        target_freq_arg,
        report_freq_arg,
        record_bag_arg,
        bag_name_arg,
        start_msg,
        monitor_node,
        record_process
    ])