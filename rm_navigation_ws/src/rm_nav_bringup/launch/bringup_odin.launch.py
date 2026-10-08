"""Odin1 mapping, relocalization, and Nav2 bringup for the real sentry."""

import os
import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _value(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _bringup(context, *unused_args, **unused_kwargs):
    mode = _value(context, 'mode')
    if mode not in ('mapping', 'relocalization', 'nav'):
        raise RuntimeError('mode must be mapping, relocalization, or nav')

    world = _value(context, 'world')
    if not re.fullmatch(r'[A-Za-z0-9_-]+', world):
        raise RuntimeError('world must contain only letters, digits, _ or -')

    world_dir = Path(_value(context, 'map_dir')).expanduser().resolve() / world
    odin_map = Path(_value(context, 'odin_map')).expanduser().resolve() if _value(context, 'odin_map') else world_dir / f'{world}.bin'
    nav_map = Path(_value(context, 'nav_map')).expanduser().resolve() if _value(context, 'nav_map') else world_dir / f'{world}.yaml'

    if mode != 'mapping' and not odin_map.is_file():
        raise RuntimeError(f'Odin relocalization map is missing: {odin_map}')
    if mode == 'nav' and not nav_map.is_file():
        raise RuntimeError(f'Nav2 occupancy map is missing: {nav_map}')
    if mode == 'nav':
        grid = yaml.safe_load(nav_map.read_text(encoding='utf-8'))
        if not grid or not (nav_map.parent / grid.get('image', '')).is_file():
            raise RuntimeError(f'Nav2 map image is missing for {nav_map}')

    world_dir.mkdir(parents=True, exist_ok=True)
    driver_share = Path(get_package_share_directory('odin_ros_driver'))
    driver_config = yaml.safe_load((driver_share / 'config' / 'control_command.yaml').read_text(encoding='utf-8'))
    keys = driver_config['register_keys']
    keys['custom_map_mode'] = 1 if mode == 'mapping' else 2
    keys['relocalization_map_abs_path'] = '' if mode == 'mapping' else str(odin_map)
    keys['mapping_result_dest_dir'] = str(world_dir)
    keys['mapping_result_file_name'] = f'{world}.bin'
    keys['sendodom'] = 1
    keys['senddtof'] = 1
    keys['sendcloudslam'] = 1 if mode == 'mapping' else 0
    # The dashboard uses the device's JPEG stream. The current driver gates
    # both raw and compressed image publication behind sendrgb.
    keys['sendrgb'] = 1
    keys['sendrgbcompressed'] = 1
    keys['sendcloudrender'] = 0
    # Nav2 and tf2 need timestamps on the host ROS clock axis.
    keys['use_host_ros_time'] = 2
    keys['tf_extra_publish_rate'] = 100
    config_path = world_dir / f'odin_{mode}_control.yaml'
    temporary_path = config_path.with_suffix('.yaml.tmp')
    temporary_path.write_text(yaml.safe_dump(driver_config, allow_unicode=True, sort_keys=False), encoding='utf-8')
    os.replace(temporary_path, config_path)

    actions = [
        LogInfo(msg=f'[Odin1] mode={mode}; device map={odin_map}; Nav2 map={nav_map}'),
        Node(
            package='odin_ros_driver', executable='host_sdk_sample', name='host_sdk_sample',
            output='screen', parameters=[{'config_file': str(config_path)}],
        ),
    ]

    if mode == 'mapping':
        actions.append(LogInfo(msg=(
            'Drive the whole robot through the area to collect the map. When finished, run: '
            'ros2 run rm_nav_bringup finish_odin_mapping.py --map-dir '
            f'{world_dir} --world {world} '
            f'--grid-resolution {_value(context, "grid_resolution")}'
        )))
        return actions

    localization_check = Node(
        package='rm_nav_bringup', executable='wait_odin_localization.py',
        name='wait_odin_localization', output='screen',
        parameters=[{'timeout_sec': float(_value(context, 'localization_timeout'))}],
    )
    actions.append(localization_check)
    if mode == 'relocalization':
        actions.append(LogInfo(msg='Move the whole robot through the mapped scene; Odin reports success via map/odom TF.'))
        return actions

    mount = _value(context, 'imu_to_base').split()
    if len(mount) != 6:
        raise RuntimeError('nav requires imu_to_base: "x y z roll pitch yaw" (measured in meters/radians)')
    try:
        [float(v) for v in mount]
    except ValueError as exc:
        raise RuntimeError('imu_to_base values must be numbers') from exc

    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=[
            '--x', mount[0], '--y', mount[1], '--z', mount[2],
            '--roll', mount[3], '--pitch', mount[4], '--yaw', mount[5],
            '--frame-id', 'imu', '--child-frame-id', 'base_link',
        ],
    ))

    navigation_dir = Path(get_package_share_directory('rm_navigation')) / 'launch'
    bringup_share = Path(get_package_share_directory('rm_nav_bringup'))
    nav_params = str(bringup_share / 'config' / 'reality' / 'nav2_params_odin.yaml')
    start_nav = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(navigation_dir / 'map_server_launch.py')),
            launch_arguments={
                'map': str(nav_map), 'params_file': nav_params,
                'use_sim_time': 'false', 'use_composition': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(navigation_dir / 'bringup_rm_navigation.py')),
            launch_arguments={
                'map': str(nav_map), 'params_file': nav_params,
                'use_sim_time': 'false', 'use_composition': 'false',
                'nav_rviz': _value(context, 'nav_rviz'),
            }.items(),
        ),
        Node(
            package='fake_vel_transform', executable='fake_vel_transform_node',
            output='screen', parameters=[{'spin_speed': 0.0, 'use_nav_wz': False}],
        ),
    ]

    def _after_localization(event, unused_context):
        if event.returncode == 0:
            return [LogInfo(msg='[Odin1] localization ready; starting Nav2')] + start_nav
        return [LogInfo(msg='[Odin1] localization failed or timed out; Nav2 was not started')]

    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=localization_check, on_exit=_after_localization,
    )))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', description='mapping, relocalization, or nav'),
        DeclareLaunchArgument('world', default_value='RMUL2026'),
        DeclareLaunchArgument('map_dir', default_value=str(Path.home() / '.ros' / 'odin_maps')),
        DeclareLaunchArgument('odin_map', default_value='', description='Optional Odin .bin map override'),
        DeclareLaunchArgument('nav_map', default_value='', description='Optional Nav2 .yaml map override'),
        # Provisional: user measured the *housing centre*, not the imu origin.
        # Approximate them as coincident until the housing-to-imu offset is
        # measured. The URDF places base_link at wheel-centre height (6 cm),
        # so housing z=45 cm above ground gives base_link -> housing z=39 cm.
        # With Odin facing backwards (yaw=pi), the inverse is below.
        DeclareLaunchArgument('imu_to_base', default_value='-0.10 0 -0.39 0 0 3.141592653589793',
                              description='Approximate imu -> base_link; calibrate housing-centre to imu offset'),
        DeclareLaunchArgument('grid_resolution', default_value='0.05'),
        DeclareLaunchArgument('sensor_height', default_value='0.45',
                              description='Legacy argument; PCD grid estimates floor height after saving'),
        DeclareLaunchArgument('localization_timeout', default_value='0.0', description='0 waits indefinitely'),
        DeclareLaunchArgument('nav_rviz', default_value='false'),
        OpaqueFunction(function=_bringup),
    ])
