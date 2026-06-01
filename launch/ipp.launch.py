from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node, PushROSNamespace
from launch.conditions import IfCondition, UnlessCondition

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    arg = DeclareLaunchArgument('use_sim_time')
    launch_description.add_action(arg)

    arg = DeclareLaunchArgument('use_scenario',
        default_value='false',
        description='Set to false during final project event, when scenario is provided by MuM')
    launch_description.add_action(arg)

    package_path = get_package_share_path('ipp')
    mapping_params_file_path = str(package_path / 'config/mapping_params.yaml')

    scenario_arg = DeclareLaunchArgument(
        name='scenario',
        default_value=str(1),
        description='The number of the scenario')
    launch_description.add_action(scenario_arg)

    group = GroupAction([
        PushROSNamespace(LaunchConfiguration('vehicle_name')),
        Node(
            executable='mapper.py',
            package='ipp',
            parameters=[
                LaunchConfiguration('mapping_params',
                                    default=mapping_params_file_path),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
        Node(
            executable='path_planner.py',
            package='ipp',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
        Node(
            executable='path_follower.py',
            package='ipp',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
        Node(executable='position_controller.py',
             package='ipp',
             parameters=[{
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
             }]),
        Node(
            executable='yaw_controller.py',
            package='ipp',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
        Node(
            executable='scenario_node',
            package='fav',
            parameters=[
                {
                    'scenario': LaunchConfiguration('scenario'),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                },
            ],
            condition=IfCondition(LaunchConfiguration('use_scenario')),
        ),
        Node(
            executable='robot_marker_publisher',
            package='fav',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
    ])
    launch_description.add_action(group)

    rviz_file_sim = str(
        get_package_share_path('ipp') / 'config/rviz.rviz'
    )
    rviz_file_experiment = str(
        get_package_share_path('ipp')
        / 'config/rviz_experiment.rviz'
    )
    group = GroupAction(
        [
            Node(
                executable='rviz2',
                package='rviz2',
                name='rviz2',
                arguments=[
                    '-d',
                    rviz_file_sim,
                    '--ros-args',
                    '--log-level',
                    'error',
                ],
                condition=IfCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                executable='rviz2',
                package='rviz2',
                name='rviz2',
                arguments=[
                    '-d',
                    rviz_file_experiment,
                    '--ros-args',
                    '--log-level',
                    'error',
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
            ),
        ]
    )
    launch_description.add_action(group)

    return launch_description
