from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.parameter_descriptions import ParameterValue
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

    launch_description.add_action(DeclareLaunchArgument(
        'planner',
        default_value='rig',
        choices=['rig', 'lawnmower', 'random'],
        description="Path planner mode: 'rig' (RIG-tree), 'lawnmower' "
                    "(non-informative coverage baseline), or 'random' "
                    '(RIG-tree without variance information). Forwarded to '
                    "path_planner.py's 'planner_type' parameter.",
    ))

    launch_description.add_action(DeclareLaunchArgument(
        'rng_seed',
        default_value='-1',
        description='Seed for the RIG-tree Sample() RNG, forwarded to '
                    "path_planner.py's 'rng_seed' parameter. -1 leaves it "
                    'unseeded.',
    ))

    launch_description.add_action(DeclareLaunchArgument(
        'planning_horizon',
        default_value='6',
        description='B (max tree node count per planning cycle), Hollinger '
                    "& Sukhatme 2014. Forwarded to path_planner.py's "
                    "'planning_horizon' parameter.",
    ))

    launch_description.add_action(DeclareLaunchArgument(
        'planning_time_s',
        default_value='2.0',
        description='Anytime wall-clock budget (s) per planning cycle. '
                    "Forwarded to path_planner.py's 'planning_time_s' "
                    'parameter.',
    ))

    launch_description.add_action(DeclareLaunchArgument(
        'roi_adaptive_enabled',
        default_value='false',
        description='Enable value-dependent region-of-interest weighting '
                    '(Hitz et al. 2017, Sec. 4.3). Forwarded to '
                    "path_planner.py's 'roi_adaptive_enabled' parameter.",
    ))

    launch_description.add_action(DeclareLaunchArgument(
        'roi_threshold',
        default_value='0.008',
        description='f_th (Hitz et al. 2017, Eq. 14), only used when '
                    'roi_adaptive_enabled. Tuned to the current scenarios '
                    '(amplitude=0.011, clip_max=0.02); re-tune if that '
                    'changes. Forwarded to path_planner.py.',
    ))

    launch_description.add_action(DeclareLaunchArgument(
        'roi_beta',
        default_value='2.0',
        description='beta (Hitz et al. 2017, Eq. 15), only used when '
                    'roi_adaptive_enabled. Forwarded to path_planner.py.',
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
        # PATH PLANNER (RIG-tree or lawnmower baseline, selected by 'planner')
        ########################################################################
        Node(
            executable='path_planner.py',
            package='ipp',
            parameters=[
                LaunchConfiguration('mapping_params',
                                    default=mapping_params_file_path),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'planner_type': LaunchConfiguration('planner'),
                    'rng_seed': ParameterValue(
                        LaunchConfiguration('rng_seed'), value_type=int),
                    'planning_horizon': ParameterValue(
                        LaunchConfiguration('planning_horizon'),
                        value_type=int),
                    'planning_time_s': ParameterValue(
                        LaunchConfiguration('planning_time_s'),
                        value_type=float),
                    'roi_adaptive_enabled': ParameterValue(
                        LaunchConfiguration('roi_adaptive_enabled'),
                        value_type=bool),
                    'roi_threshold': ParameterValue(
                        LaunchConfiguration('roi_threshold'),
                        value_type=float),
                    'roi_beta': ParameterValue(
                        LaunchConfiguration('roi_beta'), value_type=float),
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
