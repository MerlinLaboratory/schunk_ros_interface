from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def load_yaml(yaml_file_path):
    with open(yaml_file_path, 'r') as file:
        params = yaml.safe_load(file)
        return params

def launch_setup(context, *args, **kwargs):
    
    config = os.path.join(
        get_package_share_directory('schunk_hardware_interface'),
        'config',
        'params.yaml'
    )    
                   
    gripper_node = Node(
            package='schunk_hardware_interface',
            namespace="schunk",
            executable='schunk_gripper_node',
            name="gripper",
            parameters=[config],
            output="screen"
        )
    
    return [gripper_node]

def generate_launch_description():
    
    # Launch arguments
    launch_args = []
    
    # Description package
    launch_args.append(
        DeclareLaunchArgument(
            name="gripper_name",
            default_value="egk_40",
            description="The name of your gripper.",
        )
    )

    # Description package
    launch_args.append(DeclareLaunchArgument(
            name="gripper_ip",
            default_value="192.168.125.160",
            description="The static ip of your gripper.",
        )
    )
    
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
