from ament_index_python.packages import get_package_share_path
from hippo_common.launch_helper import LaunchArgsDict
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace


def generate_launch_description():
    launch_description = LaunchDescription()

    launch_description.add_action(
        DeclareLaunchArgument(
            'vehicle_name',
            default_value='bluerov00',
            description='Vehicle name used as namespace.',
        ))
    launch_description.add_action(
        DeclareLaunchArgument(
            'start_gui',
            default_value='false',
            description='Start the Gazebo GUI. Otherwise runs headless.',
        ))
    launch_description.add_action(
        DeclareLaunchArgument(
            'planner',
            default_value='rig',
            description="Path planner to run: 'rig', 'lawnmower' baseline, "
                        "or 'random' baseline.",
        ))
    launch_description.add_action(
        DeclareLaunchArgument(
            'rng_seed',
            default_value='-1',
            description='Seed for the RIG-tree Sample() RNG. -1 leaves it '
                        'unseeded.',
        ))
    launch_description.add_action(
        DeclareLaunchArgument(
            'planning_horizon',
            default_value='6',
            description='B (max tree node count per planning cycle).',
        ))
    launch_description.add_action(
        DeclareLaunchArgument(
            'planning_time_s',
            default_value='2.0',
            description='Anytime wall-clock budget (s) per planning cycle.',
        ))
    launch_description.add_action(
        DeclareLaunchArgument(
            'roi_adaptive_enabled',
            default_value='false',
            description='Enable value-dependent region-of-interest '
                        'weighting (Hitz et al. 2017, Sec. 4.3).',
        ))
    launch_description.add_action(
        DeclareLaunchArgument(
            'roi_threshold',
            default_value='0.008',
            description='f_th (Hitz et al. 2017, Eq. 14), only used when '
                        'roi_adaptive_enabled. Tuned to the current '
                        'scenarios (amplitude=0.011, clip_max=0.02).',
        ))
    launch_description.add_action(
        DeclareLaunchArgument(
            'roi_beta',
            default_value='2.0',
            description='beta (Hitz et al. 2017, Eq. 15), only used when '
                        'roi_adaptive_enabled.',
        ))

    scalar_field_sim_path = get_package_share_path('scalar_field_sim')
    scalar_field_belief_path = get_package_share_path('scalar_field_belief')

    launch_description.add_action(
        DeclareLaunchArgument(
            'scenario_path',
            default_value=str(scalar_field_sim_path /
                              'scenarios/clustered_field_2.toml'),
        ))
    launch_description.add_action(
        DeclareLaunchArgument(
            'belief_params',
            default_value=str(scalar_field_belief_path /
                              'config/belief_params_default.yaml'),
        ))

    hippo_sim = get_package_share_path('hippo_sim')

    ############################################################################
    # GAZEBO
    ############################################################################
    args = LaunchArgsDict()
    args.add('start_gui')
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(hippo_sim / 'launch/start_gazebo.launch.py')),
            launch_arguments=args.items(),
        ))

    ############################################################################
    # SPAWN BLUEROV
    ############################################################################
    args = LaunchArgsDict()
    args.add_vehicle_name()
    args['use_sim_time'] = 'true'
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(hippo_sim / 'launch/spawn_bluerov.launch.py')),
            launch_arguments=args.items(),
        ))

    ############################################################################
    # IPP
    ############################################################################
    args = LaunchArgsDict()
    args.add('vehicle_name')
    args.add('planner')
    args.add('rng_seed')
    args.add('planning_horizon')
    args.add('planning_time_s')
    args.add('roi_adaptive_enabled')
    args.add('roi_threshold')
    args.add('roi_beta')
    args['use_sim_time'] = 'true'
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(get_package_share_path('ipp') / 'launch/ipp.launch.py')),
            launch_arguments=args.items(),
        ))

    ############################################################################
    # RVIZ VISUALS
    ############################################################################
    launch_description.add_action(
        GroupAction([
            PushROSNamespace(LaunchConfiguration('vehicle_name')),
            Node(
                executable='robot_marker_publisher',
                package='ipp_rviz_visuals',
                parameters=[{
                    'use_sim_time': True
                }],
            ),
        ]))

    ############################################################################
    # SCALAR FIELD SIM (field server + ground-truth visualization)
    ############################################################################
    args = LaunchArgsDict()
    args.add('scenario_path')
    args['use_sim_time'] = 'true'
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(scalar_field_sim_path / 'launch/field_sim.launch.py')),
            launch_arguments=args.items(),
        ))

    ############################################################################
    # MEASUREMENT PIPELINE + GP BELIEF (under vehicle namespace)
    ############################################################################
    launch_description.add_action(
        GroupAction([
            PushROSNamespace(LaunchConfiguration('vehicle_name')),
            Node(
                package='scalar_field_sim',
                executable='ir_measurement_sim_node.py',
                name='ir_measurement_sim',
                parameters=[{
                    'use_sim_time': True
                }],
                output='screen',
                emulate_tty=True,
            ),
            Node(
                package='scalar_field_belief',
                executable='scalar_field_belief_node.py',
                name='scalar_field_belief',
                parameters=[
                    LaunchConfiguration('belief_params'),
                    {
                        'use_sim_time': True
                    },
                ],
                output='screen',
                emulate_tty=True,
            ),
        ]))

    return launch_description
