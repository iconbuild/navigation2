# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
#from launch_ros.actions import PushROSNamespace
from launch_ros.descriptions import ParameterFile, ComposableNode
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    log_level = LaunchConfiguration('log_level')

    keepout_mask_params_file = LaunchConfiguration('keepout_mask_params_file')
    keepout_mask_yaml_file = LaunchConfiguration('keepout_mask')

    speed_params_file = LaunchConfiguration('speed_params_file')
    speed_mask_yaml_file = LaunchConfiguration('speed_mask')

    preferred_lanes_params_file = LaunchConfiguration('preferred_lanes_params_file')
    preferred_lanes_mask_yaml_file = LaunchConfiguration('preferred_lanes_mask')

    horns_zone_params_file = LaunchConfiguration('horns_zone_params_file')
    horns_zone_mask_yaml_file = LaunchConfiguration('horns_zone_mask')


    # Keepout zones
    keepout_lifecycle_nodes = ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server']
    # Make re-written yaml
    keepout_mask_param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': keepout_mask_yaml_file}

    keepout_mask_configured_params = RewrittenYaml(
        source_file=keepout_mask_params_file,
        root_key=namespace,
        param_rewrites=keepout_mask_param_substitutions,
        convert_types=True)

    declare_keepout_mask_params_file_cmd = DeclareLaunchArgument(
            'keepout_mask_params_file',
            default_value='/nav2_config/keepout_params.yaml',
            description='Full path to the ROS2 parameters file to use')

    declare_keepout_mask_yaml_file_cmd = DeclareLaunchArgument(
            'keepout_mask',
            default_value='/maps/keepout_mask.yaml',
            description='Full path to filter mask yaml file to load')

    # # Speed zones

    # speed_lifecycle_nodes = ['speed_filter_mask_server', 'speed_costmap_filter_info_server']

    # # Make re-written yaml
    # speed_mask_param_substitutions = {
    #     'use_sim_time': use_sim_time,
    #     'yaml_filename': speed_mask_yaml_file}

    # speed_mask_configured_params = RewrittenYaml(
    #     source_file=speed_params_file,
    #     root_key=namespace,
    #     param_rewrites=speed_mask_param_substitutions,
    #     convert_types=True)
    

    # declare_speed_mask_params_file_cmd = DeclareLaunchArgument(
    #         'speed_params_file',
    #         default_value='/nav2_config/speed_params.yaml',
    #         description='Full path to the ROS2 parameters file to use')

    # declare_speed_mask_yaml_file_cmd = DeclareLaunchArgument(
    #         'speed_mask',
    #         default_value='/maps/speed_mask.yaml',
    #         description='Full path to filter mask yaml file to load')

    # Preferred Lanes
    
    preferred_lanes_lifecycle_nodes = ['preferred_lanes_filter_mask_server','preferred_lanes_costmap_filter_info_server']

    ## Make re-written yaml
    preferred_lanes_mask_param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': preferred_lanes_mask_yaml_file}

    preferred_lanes_mask_configured_params = RewrittenYaml(
        source_file=preferred_lanes_params_file,
        root_key=namespace,
        param_rewrites=preferred_lanes_mask_param_substitutions,
        convert_types=True)
    

    declare_preferred_lanes_mask_params_file_cmd = DeclareLaunchArgument(
            'preferred_lanes_params_file',
            default_value='/nav2_config/preferred_lanes_params.yaml',
            description='Full path to the ROS2 parameters file to use')

    declare_preferred_lanes_mask_yaml_file_cmd = DeclareLaunchArgument(
            'preferred_lanes_mask',
            default_value='/maps/preferred_lanes_mask.yaml',
            description='Full path to filter mask yaml file to load')
    
    # Binary zones
    
    horns_zone_lifecycle_nodes = ['horns_zone_filter_mask_server', 'horns_zone_costmap_filter_info_server']

    ## Make re-written yaml
    horns_zone_mask_param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': horns_zone_mask_yaml_file}

    horns_zone_mask_configured_params = RewrittenYaml(
        source_file=horns_zone_params_file,
        root_key=namespace,
        param_rewrites=horns_zone_mask_param_substitutions,
        convert_types=True)
    

    declare_horns_zone_mask_params_file_cmd = DeclareLaunchArgument(
            'horns_zone_params_file',
            default_value='/nav2_config/horns_zone_params.yaml',
            description='Full path to the ROS2 parameters file to use')

    declare_horns_zone_mask_yaml_file_cmd = DeclareLaunchArgument(
            'horns_zone_mask',
            default_value='/maps/horns_zone_mask.yaml',
            description='Full path to filter mask yaml file to load')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack',
    )


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
    #     description='Full path to the ROS2 parameters file to use for all launched nodes',
    # )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )


    # Specify the actions
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            # Keepout Filter
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='keepout_filter_mask_server',
                parameters=[keepout_mask_configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::CostmapFilterInfoServer',
                name='keepout_costmap_filter_info_server',
                parameters=[keepout_mask_configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='keepout_lifecycle_manager_costmap_filters',
                parameters=[{'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': keepout_lifecycle_nodes}]),
            # Speed Filter
            #ComposableNode(
            #    package='nav2_map_server',
            #    plugin='nav2_map_server::MapServer',
            #    name='speed_filter_mask_server',
            #    parameters=[speed_mask_configured_params],
            #    remappings=remappings),
            #ComposableNode(
            #    package='nav2_map_server',
            #    plugin='nav2_map_server::CostmapFilterInfoServer',
            #    name='speed_costmap_filter_info_server',
            #    parameters=[speed_mask_configured_params],
            #    remappings=remappings),
            #ComposableNode(
            #    package='nav2_lifecycle_manager',
            #    plugin='nav2_lifecycle_manager::LifecycleManager',
            #    name='speed_lifecycle_manager_costmap_filters',
            #    parameters=[{'use_sim_time': use_sim_time,
            #         'autostart': autostart,
            #         'node_names': speed_lifecycle_nodes}]),

            # Horns Zone Filter
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='horns_zone_filter_mask_server',
                parameters=[horns_zone_mask_configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::CostmapFilterInfoServer',
                name='horns_zone_costmap_filter_info_server',
                parameters=[horns_zone_mask_configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='horns_zone_lifecycle_manager_costmap_filters',
                parameters=[{'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'node_names': horns_zone_lifecycle_nodes}]),

            # Preferred Lanes Filter
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='preferred_lanes_filter_mask_server',
                parameters=[preferred_lanes_mask_configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::CostmapFilterInfoServer',
                name='preferred_lanes_costmap_filter_info_server',
                parameters=[preferred_lanes_mask_configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='preferred_lanes_lifecycle_manager_costmap_filters',
                parameters=[{'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'node_names': preferred_lanes_lifecycle_nodes}])
        ],
    )

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_keepout_mask_params_file_cmd)
    ld.add_action(declare_keepout_mask_yaml_file_cmd)
    # ld.add_action(declare_speed_mask_params_file_cmd)
    # ld.add_action(declare_speed_mask_yaml_file_cmd)
    ld.add_action(declare_preferred_lanes_mask_params_file_cmd)
    ld.add_action(declare_preferred_lanes_mask_yaml_file_cmd)
    ld.add_action(declare_horns_zone_mask_params_file_cmd)
    ld.add_action(declare_horns_zone_mask_yaml_file_cmd)
    # Add the actions to launch all of the navigation nodes
    # ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
