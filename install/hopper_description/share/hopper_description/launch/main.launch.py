from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('hopper_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name', default='$(find  hopper_description)/worlds/low_gravity.world')
    
    declare_use_sim_time = DeclareLaunchArgument(
        name = 'use_sim_time' , 
        default_value = 'True' , 
        description = 'Use simulation (Gazebo) clock if true')
      
      # Launch your robot spawn and control
    spawn_monoped_launch= IncludeLaunchDescription(PythonLaunchDescriptionSource([get_package_share_directory('hopper_description'), '/launch/spawn_monoped.launch.py']))
    monoped_control_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([get_package_share_directory('hopper_description'), '/launch/monoped_control.launch.py']))

    return LaunchDescription([
        spawn_monoped_launch,
        monoped_control_launch,
        declare_use_sim_time
    ])

if __name__ == '__main__':
    generate_launch_description()