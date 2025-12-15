import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'eliko_driver'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    config_path = os.path.join(pkg_share, 'config', 'eliko.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config_eliko.rviz')

    # Parse YAML to control launch flow
    config_dict = {}
    try:
        with open(config_path, 'r') as f:
            yaml_content = yaml.safe_load(f)
            config_dict = yaml_content['eliko_driver_node']['ros__parameters']
    except Exception as e:
        print(f"Error reading YAML: {e}. Using defaults.")

    # Extract flags
    run_viz = config_dict.get('viz', False)
    record_bag = config_dict.get('record_bag', False)
    debug_mode = config_dict.get('debug_mode', False)

    # 1. Main Driver Node
    driver_node = Node(
        package=pkg_name,
        executable='eliko_driver',
        name='eliko_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_path,
        ], 
        prefix=['xterm -e gdb -ex run --args'] if debug_mode else []
    )

    # 2. RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
    )

    # 3. Rosbag Record
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen'
    )

    # Build description
    ld = LaunchDescription()
    ld.add_action(driver_node)
    
    if run_viz:
        ld.add_action(rviz_node)
        
    if record_bag:
        ld.add_action(bag_record)

    return ld