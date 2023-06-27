import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    qlearn_params_file = os.path.join(get_package_share_directory('hopper_description'), 'hopper_training','config','qlearn_params.yaml')


    # Load the YAML configuration file using the ros2param command
    load_qlearn_params = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/monoped_gym', qlearn_params_file],
    )

    # Launch the training system
    start_training_node = Node(
        package='hopper_description/hopper_training',
        executable='start_training_v2.py',
        name='monoped_gym',
        output='screen'
    )

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
        'qlearn_params_file',
        default_value=qlearn_params_file,
        description='Path to qlearn_params.yaml file'
    ),
        start_training_node,
        load_qlearn_params,
    ])
