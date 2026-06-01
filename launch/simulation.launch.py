from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    fav_sim = str(
        get_package_share_path('fav') / 'launch/simulation.launch.py')
    ipp = str(
        get_package_share_path('ipp') / 'launch/ipp.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fav_sim),
            launch_arguments={
                'vehicle_name': 'bluerov00',
                'start_gui': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ipp),
            launch_arguments={
                'vehicle_name': 'bluerov00',
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
