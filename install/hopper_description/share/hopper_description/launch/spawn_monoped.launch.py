from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    spawn_robot_tools_pkg = get_package_share_directory('spawn_robot_tools_pkg')
    hopper_description_pkg = get_package_share_directory('hopper_description')

    spawn_robot_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([spawn_robot_tools_pkg, '/launch/spawn_robot_urdf.launch.py']),
        launch_arguments={
            'x': '0.0',
            'y': '0.0',
            'z': '1.0',
            'roll': '0',
            'pitch': '0',
            'yaw': '0.0',
            'urdf_robot_file': hopper_description_pkg + '/urdf/hopper.urdf.xacro',
            'robot_name': 'monoped'
        }.items()
    )

    return LaunchDescription([spawn_robot_urdf_launch])
