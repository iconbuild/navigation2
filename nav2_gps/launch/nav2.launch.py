
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    # gps_wpf_dir = get_package_share_directory("nav2_gps")

    # params_dir = os.path.join(gps_wpf_dir, "config")
    # maps_dir = os.path.join(gps_wpf_dir, "maps")
    ## behaviors_dir = os.path.join(gps_wpf_dir, "behaviors")
    # nav2_maps = os.path.join(maps_dir, "map.yaml")
    # nav2_params = os.path.join(params_dir, "nav2_params.yaml")


    params_dir = os.path.join(bringup_dir, "config")
    maps_dir = os.path.join(bringup_dir, "maps")
    # behaviors_dir = os.path.join(bringup_dir, "behaviors")
    nav2_maps = os.path.join(maps_dir, "map.yaml") 
    nav2_params = os.path.join(params_dir, "nav2_params.yaml")


    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use sim time')


    odom_to_tf_cmd = Node(
        package="odom_to_tf_ros2",
        executable="odom_to_tf",
        parameters=[
            {"frame_id": 'odom'},
            {"child_frame_id": 'gps_link'},
            {"odom_topic": '/fixposition/odometry_enu'},
            ],
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": "True",
            "map": nav2_maps,
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz)
    )

    # Create the launch description and populate
    ld = LaunchDescription()


    # navigation2 launch
    ld.add_action(odom_to_tf_cmd)
    ld.add_action(navigation2_cmd)

    # viz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    return ld
