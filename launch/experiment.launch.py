from ament_index_python.packages import get_package_share_path
from hippo_common.launch_helper import LaunchArgsDict
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    launch_description.add_action(
        DeclareLaunchArgument(
            'vehicle_name',
            default_value='bluerov00',
            description='Vehicle name used as namespace.',
        ))
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
                    '(RIG-tree without variance information).',
    ))
    launch_description.add_action(DeclareLaunchArgument(
        'rng_seed',
        default_value='-1',
        description='Seed for the RIG-tree Sample() RNG. -1 leaves it '
                    'unseeded.',
    ))
    launch_description.add_action(DeclareLaunchArgument(
        'planning_horizon',
        default_value='6',
        description='B (max tree node count per planning cycle).',
    ))
    launch_description.add_action(DeclareLaunchArgument(
        'planning_time_s',
        default_value='2.0',
        description='Anytime wall-clock budget (s) per planning cycle.',
    ))
    launch_description.add_action(DeclareLaunchArgument(
        'roi_adaptive_enabled',
        default_value='false',
        description='Enable value-dependent region-of-interest weighting '
                    '(Hitz et al. 2017, Sec. 4.3).',
    ))
    launch_description.add_action(DeclareLaunchArgument(
        'roi_threshold',
        default_value='0.008',
        description='f_th (Hitz et al. 2017, Eq. 14), only used when '
                    'roi_adaptive_enabled.',
    ))
    launch_description.add_action(DeclareLaunchArgument(
        'roi_beta',
        default_value='2.0',
        description='beta (Hitz et al. 2017, Eq. 15), only used when '
                    'roi_adaptive_enabled.',
    ))

    belief_params_file_path = str(
        get_package_share_path('scalar_field_belief') /
        'config/belief_params_default.yaml')
    launch_description.add_action(DeclareLaunchArgument(
        'belief_params',
        default_value=belief_params_file_path,
        description='Params file for scalar_field_belief_node.py. Its '
                    'x_min/x_max/y_min/y_max must match the real tank and '
                    "ipp's mapping_params.yaml tank_size_x/y.",
    ))

    ############################################################################
    # IPP (mapper, path_planner, path_follower, position_controller,
    # yaw_controller, obstacle_publisher, rviz)
    ############################################################################
    args = LaunchArgsDict()
    args.add('vehicle_name')
    args.add('scenario')
    args.add('planner')
    args.add('rng_seed')
    args.add('planning_horizon')
    args.add('planning_time_s')
    args.add('roi_adaptive_enabled')
    args.add('roi_threshold')
    args.add('roi_beta')
    args['use_sim_time'] = 'false'
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(get_package_share_path('ipp') / 'launch/ipp.launch.py')),
            launch_arguments=args.items(),
        ))

    ############################################################################
    # GP BELIEF
    # The real IR sensor driver is not part of this workspace and has to be
    # started separately. It must publish ir_measurement (ScalarMeasurement,
    # frame_id='map') and offer a trigger_ir_measurement (std_srvs/Trigger)
    # service, both in this same vehicle_name namespace.
    ############################################################################
    launch_description.add_action(
        GroupAction([
            PushROSNamespace(LaunchConfiguration('vehicle_name')),
            Node(
                package='scalar_field_belief',
                executable='scalar_field_belief_node.py',
                name='scalar_field_belief',
                parameters=[
                    LaunchConfiguration('belief_params'),
                    {
                        'use_sim_time': False,
                    },
                ],
                output='screen',
                emulate_tty=True,
            ),
        ]))

    ############################################################################
    # RVIZ VISUALS (tank mesh + BlueROV mesh, driven by vision_pose_cov)
    ############################################################################
    launch_description.add_action(
        GroupAction([
            PushROSNamespace(LaunchConfiguration('vehicle_name')),
            Node(
                executable='robot_marker_publisher',
                package='ipp_rviz_visuals',
                parameters=[{
                    'use_sim_time': False
                }],
            ),
        ]))

    return launch_description
