import os
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def _params_filename(prefix, suffix):
    return [
        TextSubstitution(text=prefix),
        suffix,
        TextSubstitution(text='.yaml')
    ]


_TRUE_STRINGS = "['true', '1', 'yes', 'on']"


def _is_true_expression(name):
    return ["'", LaunchConfiguration(name), "'.lower() in ", _TRUE_STRINGS]


def _if_true(name):
    return IfCondition(PythonExpression(_is_true_expression(name)))


def _if_slam_needed():
    return IfCondition(PythonExpression(
        _is_true_expression('slam') + [' or '] + _is_true_expression('nav2')
    ))


def _resolve_slam_map_file(context):
    """slam_toolbox reads map_file_name as given, so a relative path would depend on the launch's working
    directory. Resolve a relative one against the 41068_ignition_bringup package and hand slam_toolbox a
    copy of the params file with the absolute path."""
    params_path = LaunchConfiguration('slam_params_file_in').perform(context)
    with open(params_path) as f:
        params = yaml.safe_load(f) or {}
    package_dir = FindPackageShare('41068_ignition_bringup').perform(context)
    changed = False
    for node in params.values():
        ros_params = node.get('ros__parameters', {}) if isinstance(node, dict) else {}
        map_file = ros_params.get('map_file_name')
        if isinstance(map_file, str) and map_file and not os.path.isabs(map_file):
            ros_params['map_file_name'] = os.path.join(package_dir, map_file)
            changed = True
    if not changed:
        return [SetLaunchConfiguration('slam_params_file_resolved', params_path)]
    out = tempfile.NamedTemporaryFile('w', prefix='slam_params_', suffix='.yaml', delete=False)
    with out:
        yaml.safe_dump(params, out)
    return [SetLaunchConfiguration('slam_params_file_resolved', out.name)]


def generate_launch_description():

    ld = LaunchDescription()

    config_path = PathJoinSubstitution([
        FindPackageShare('41068_ignition_bringup'),
        'config'
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )

    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace, e.g. husky1 or parrot1'
    )

    config_filename_suffix = LaunchConfiguration('config_filename_suffix')
    config_filename_suffix_launch_arg = DeclareLaunchArgument(
        'config_filename_suffix',
        default_value='',
        description='Robot filename suffix, e.g. _husky1 or _parrot1'
    )

    slam_launch_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Flag to launch SLAM Toolbox'
    )

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='False',
        description='Flag to launch Nav2. Nav2 also starts SLAM.'
    )

    slam_params_file_in = PathJoinSubstitution([
        config_path,
        _params_filename('slam_params', config_filename_suffix)
    ])
    slam_params_file = LaunchConfiguration('slam_params_file_resolved')

    nav2_params_file = PathJoinSubstitution([
        config_path,
        _params_filename('nav2_params', config_filename_suffix)
    ])

    # Start Simultaneous Localisation and Mapping (SLAM).
    # Use the official slam_toolbox launch file, but wrap it in the robot
    # namespace and remap TF to relative names so it uses /<robot>/tf.
    slam = GroupAction(scoped=True, condition=_if_slam_needed(), actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        # Some packages publish/subscribe absolute /map topics even when
        # started under a namespace. Force them back to relative names so
        # they become /<robot>/map and /<robot>/map_updates.
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/map_updates', dst='map_updates'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
                # Keep slam_toolbox lifecycle handling inside its own launch
                # file. This avoids mismatches between executable/node type
                # across ROS distributions.
                'use_lifecycle_manager': 'True',
                'autostart': 'True',
            }.items()
        )
    ])

    # Start Navigation Stack.
    # We push the namespace ourselves rather than relying on nav2_bringup's
    # use_namespace handling, which differs across distro/package versions.
    navigation_group = GroupAction(scoped=True, condition=_if_true('nav2'), actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        # Some packages publish/subscribe absolute /map topics even when
        # started under a namespace. Force them back to relative names so
        # they become /<robot>/map and /<robot>/map_updates.
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/map_updates', dst='map_updates'),
        # Nav2's final output (the velocity smoother's cmd_vel_smoothed, which nav2_bringup remaps to cmd_vel)
        # goes to the water guard instead, which gates it onto cmd_vel. Rules set here come before
        # nav2_bringup's own remaps, so this one wins; nothing else in Nav2 uses cmd_vel_smoothed.
        SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_nav_smoothed'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
            }.items()
        )
    ])

    navigation = navigation_group

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(namespace_launch_arg)
    ld.add_action(config_filename_suffix_launch_arg)
    ld.add_action(slam_launch_arg)
    ld.add_action(nav2_launch_arg)
    ld.add_action(SetLaunchConfiguration('slam_params_file_in', slam_params_file_in))
    ld.add_action(OpaqueFunction(function=_resolve_slam_map_file))
    ld.add_action(slam)
    ld.add_action(navigation)

    return ld
