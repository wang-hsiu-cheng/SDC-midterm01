import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
    arguments = [
        DeclareLaunchArgument('data_path', default_value='/root/data/Track2', description='Track directory'),
        DeclareLaunchArgument('bag_path', default_value='', description='Empty = <data_path>/<track dir name>.db3'),
        DeclareLaunchArgument('ply_path', default_value=PathJoinSubstitution([LaunchConfiguration('data_path'), 'raw_ply'])),
        DeclareLaunchArgument('init_guess_file', default_value=PathJoinSubstitution([LaunchConfiguration('data_path'), 'init_guess.txt'])),

        DeclareLaunchArgument('save_path', default_value='/root/midterm/src/localization/result/'),
        DeclareLaunchArgument('save_file', default_value='Track2_result.txt'),

        DeclareLaunchArgument('params_file', default_value='',
                              description='Empty = config/localization_params_<data_path 目錄名>.yaml'),
        DeclareLaunchArgument('max_frames', default_value='0', description='只處理前 N 幀；0 = 全部'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    def create_nodes(context):
        data_path = LaunchConfiguration('data_path').perform(context).rstrip('/')
        bag_path = LaunchConfiguration('bag_path').perform(context) or os.path.join(
            data_path, os.path.basename(data_path) + '.db3')
        # 每個 track 一份完整參數檔（localization_params_Track1/2/3.yaml）
        params_file_path = LaunchConfiguration('params_file').perform(context) or os.path.join(
            pkg_share, 'config', 'localization_params_' + os.path.basename(data_path) + '.yaml')
        if not os.path.exists(params_file_path):
            raise RuntimeError('找不到參數檔 %s；請新增該 track 的 yaml 或用 params_file:=<path> 指定' % params_file_path)

        localization_node = Node(
            package='localization', executable='localization',
            name='image_point_cloud_localizer', output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[
                params_file_path,
                {
                    'use_sim_time': False,
                    'map_path': LaunchConfiguration('data_path'),
                    'map_file': 'map.pcd',
                    'map_cache_file': 'map_downsampled_1m.pcd',
                    'bag_path': bag_path,
                    'ply_path': LaunchConfiguration('ply_path'),
                    'init_guess_file': LaunchConfiguration('init_guess_file'),
                    'save_path': LaunchConfiguration('save_path'),
                    'save_file': LaunchConfiguration('save_file'),
                    'max_frames': ParameterValue(LaunchConfiguration('max_frames'), value_type=int),
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