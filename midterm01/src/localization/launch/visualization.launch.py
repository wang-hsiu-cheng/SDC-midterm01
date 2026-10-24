from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
    rviz_config = os.path.join(
        pkg_share,
        'config',
        'visualization.rviz'
    )

    declare_data_path_cmd = DeclareLaunchArgument('data_path', default_value='/root/data/Track1', description='')
    declare_pose_file_cmd = DeclareLaunchArgument('pose_file', default_value='/root/midterm/src/localization/result/Track1_result.txt', description='')

    visualization_node = Node(
        package='localization',
        executable='visualization',
        name='visualization_node',
        output='screen',
        parameters=[{
            'map_file': PathJoinSubstitution([LaunchConfiguration('data_path'), 'map.pcd']),
            'pose_file': LaunchConfiguration('pose_file'),
            'pcd_folder': PathJoinSubstitution([LaunchConfiguration('data_path'), 'raw_ply']),
        }]
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='display_result',
    #     output='screen',
    #     arguments=['-d', rviz_config]
    # )

    return LaunchDescription([
        declare_data_path_cmd,
        declare_pose_file_cmd,
        visualization_node
        # rviz_node
    ])