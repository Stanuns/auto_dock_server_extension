from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    param_config = os.path.join(
        get_package_share_directory('auto_dock_server_extension'), 'params', 'auto_dock.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='auto_dock_server_extension', 
            executable='remote_auto_dock_server', 
            name='remote_auto_dock_server_node',
            output='screen',
            parameters=[param_config]
        )
    ])