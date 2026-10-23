import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
    default_params_file = os.path.join(pkg_share, 'config', 'localization_params.yaml')
    arguments = [
        DeclareLaunchArgument('data_path', default_value='/root/data/Track1', description='Track directory'),
        DeclareLaunchArgument('bag_path', default_value='', description='Optional .db3 bag'),
        DeclareLaunchArgument('ply_path', default_value=PathJoinSubstitution([LaunchConfiguration('data_path'), 'raw_ply'])),
        DeclareLaunchArgument('init_guess_file', default_value=PathJoinSubstitution([LaunchConfiguration('data_path'), 'init_guess.txt'])),

        DeclareLaunchArgument('save_path', default_value='/root/midterm/src/localization/result/'),
        DeclareLaunchArgument('save_file', default_value='Track1_result.txt'),

        DeclareLaunchArgument('params_file', default_value=default_params_file, description='Path to yaml parameters file'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    def create_nodes(context):
        params_file_path = LaunchConfiguration('params_file').perform(context)

        localization_node = Node(
            package='localization', executable='localization',
            name='image_point_cloud_localizer', output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[
                params_file_path,  # 載入 YAML 完整超參數
                {
                    'use_sim_time': False,
                    'map_path': LaunchConfiguration('data_path'),
                    'map_file': 'map.pcd',
                    'map_cache_file': 'map_downsampled_1m.pcd',
                    'bag_path': LaunchConfiguration('bag_path'),
                    'ply_path': LaunchConfiguration('ply_path'),
                    'init_guess_file': LaunchConfiguration('init_guess_file'),
                    'save_path': LaunchConfiguration('save_path'),
                    'save_file': LaunchConfiguration('save_file'),
                }
            ],
        )
        rviz_node = Node(
            package='rviz2', executable='rviz2', name='display_result', output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'localization.rviz')],
            parameters=[{'use_sim_time': False}],
        )
        return [localization_node, rviz_node]

    return LaunchDescription(arguments + [OpaqueFunction(function=create_nodes)])