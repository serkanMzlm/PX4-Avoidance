import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os.path import join as Path

config_file_path = get_package_share_directory('config')
config = os.path.join(config_file_path, 'config', 'params.yml')

local_planner = Node(
    package='local_planner',
    executable='local_planner_node',
    parameters=[config],
)

point_cloud = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/realsense_d435i/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked']
    )

def generate_launch_description():
    return LaunchDescription([   
        # local_planner,
        point_cloud
    ]) 

