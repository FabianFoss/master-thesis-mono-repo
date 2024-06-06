import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the map and parameters file
    map_path = LaunchConfiguration('map', default=os.path.join(
        "/home/fabianfossbudal/repos/master-thesis-mono-repo/nav2_ws/src/map/",
        'rom_gf.yaml'))

    params_file_path = LaunchConfiguration('params_file', default=os.path.join(
        '/home/fabianfossbudal/repos/master-thesis-mono-repo/nav2_ws/src/config/', 'nav2_params_scan.yaml'))

    # Declare launch arguments
    declare_map_path = DeclareLaunchArgument(
        'map',
        default_value=map_path,
        description='Full path to the map file to load')

    declare_params_file_path = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path,
        description='Full path to the navigation2 parameters file')

    # Include the nav2 bringup launch file
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ]),
        launch_arguments={
            'map': map_path,
            'params_file': params_file_path
        }.items()
    )

    return LaunchDescription([
        declare_map_path,
        declare_params_file_path,
        nav2_bringup
    ])
