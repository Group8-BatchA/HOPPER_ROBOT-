from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument for the monoped configuration file
    monoped_config = DeclareLaunchArgument(
        'monoped_config',
        default_value='$(find hopper_description)/config/monoped.yaml',
        description='Path to monoped configuration file'
    )

    # Load the monoped configuration file using the ros2param command
    load_monoped_config = ExecuteProcess(
        cmd=['ros2', 'param', 'load', 'monoped', 
             LaunchConfiguration('monoped_config')
        ],
        output='screen'
    )

    # Launch the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_monoped',
        output='screen',
        parameters=[{'publish_frequency': 20.0, 'ignore_timestamp': True, 'tf_prefix': 'monoped'}],
        remappings=[('/joint_states', '/monoped/joint_states')]
    )

    # Launch the controller spawner node
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        output='screen',
        arguments=[
            '--namespace=/monoped',
            'joint_state_controller',
            'haa_joint_position_controller',
            'hfe_joint_position_controller',
            'kfe_joint_position_controller',
            '--shutdown-timeout', '3'
        ]
    )

    return LaunchDescription([
        monoped_config,
        load_monoped_config,
        robot_state_publisher,
        controller_spawner
    ])
