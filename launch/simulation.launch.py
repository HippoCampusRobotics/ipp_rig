from ament_index_python.packages import get_package_share_path
from hippo_common.launch_helper import LaunchArgsDict
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace


def generate_launch_description():
    launch_description = LaunchDescription()

    launch_description.add_action(DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov00',
        description='Vehicle name used as namespace.',
    ))
    launch_description.add_action(DeclareLaunchArgument(
        'start_gui',
        default_value='false',
        description='Start the Gazebo GUI. Otherwise runs headless.',
    ))

    hippo_sim = get_package_share_path('hippo_sim')

    ############################################################################
    # GAZEBO
    ############################################################################
    args = LaunchArgsDict()
    args.add('start_gui')
    launch_description.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(hippo_sim / 'launch/start_gazebo.launch.py')),
        launch_arguments=args.items(),
    ))

    ############################################################################
    # SPAWN BLUEROV
    ############################################################################
    args = LaunchArgsDict()
    args.add_vehicle_name()
    args['use_sim_time'] = 'true'
    launch_description.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(hippo_sim / 'launch/spawn_bluerov.launch.py')),
        launch_arguments=args.items(),
    ))

    ############################################################################
    # IPP
    ############################################################################
    args = LaunchArgsDict()
    args.add('vehicle_name')
    args['use_sim_time'] = 'true'
    launch_description.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('ipp') / 'launch/ipp.launch.py')),
        launch_arguments=args.items(),
    ))

    ############################################################################
    # RVIZ VISUALS
    ############################################################################
    launch_description.add_action(GroupAction([
        PushROSNamespace(LaunchConfiguration('vehicle_name')),
        Node(
            executable='robot_marker_publisher',
            package='ipp_rviz_visuals',
            parameters=[{'use_sim_time': True}],
        ),
    ]))

    return launch_description
