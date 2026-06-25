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

    launch_description.add_action(DeclareLaunchArgument('vehicle_name'))
    launch_description.add_action(DeclareLaunchArgument('use_sim_time'))

    launch_description.add_action(DeclareLaunchArgument(
        'scenario',
        default_value='',
        description='Name of a scenario YAML in ipp/config/ (e.g. scenario_1). '
                    'Leave empty to run without obstacles.',
    ))

    package_path = get_package_share_path('ipp')
    mapping_params_file_path = str(package_path / 'config/mapping_params.yaml')

    group = GroupAction([
        PushROSNamespace(LaunchConfiguration('vehicle_name')),

        ########################################################################
        # MAPPER
        ########################################################################
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

        ########################################################################
        # PATH PLANNER
        ########################################################################
        Node(
            executable='path_planner.py',
            package='ipp',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'planning_horizon': 0.5,
                    'delta': 0.2,
                    'near_radius': 0.3,
                    'planning_time_s': 0.5,
                    'goal_tolerance': 0.15,
                    'robot_z': -0.5,
                    'replan_timeout_s': 20.0,
                },
            ],
        ),

        ########################################################################
        # PATH FOLLOWER
        ########################################################################
        Node(
            executable='path_follower.py',
            package='ipp',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),

        ########################################################################
        # POSITION CONTROLLER
        ########################################################################
        Node(
            executable='position_controller.py',
            package='ipp',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        ########################################################################
        # YAW CONTROLLER
        ########################################################################
        Node(
            executable='yaw_controller.py',
            package='ipp',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),

        ########################################################################
        # OBSTACLE PUBLISHER
        ########################################################################
        Node(
            executable='obstacle_publisher',
            package='ipp',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'scenario': LaunchConfiguration('scenario'),
                },
            ],
        ),
    ])
    launch_description.add_action(group)

    ############################################################################
    # RVIZ
    ############################################################################
    rviz_file_sim = str(
        get_package_share_path('ipp') / 'config/rviz.rviz'
    )
    rviz_file_experiment = str(
        get_package_share_path('ipp') / 'config/rviz_experiment.rviz'
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
