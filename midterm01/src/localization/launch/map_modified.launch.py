"""地圖前處理：ROI 裁切（可選）＋體素降採樣，輸出 binary PCD 快取。

範例：
  ros2 launch localization map_modified.launch.py data_path:=/root/data/Track3
  ros2 launch localization map_modified.launch.py data_path:=/root/data/Track1 voxel_leaf_size:=0.5 \
      roi_min:="-5100,1900,50" roi_max:="-4300,2500,200"
output_file 留空時自動命名為 <data_path>/map_downsampled_<leaf>m.pcd（leaf=1 → map_downsampled_1m.pcd，
即 localization.launch.py 預設的 map_cache_file）。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def parse_xyz(text, name):
    if not text.strip():
        return []
    values = [float(v) for v in text.split(',')]
    if len(values) != 3:
        raise ValueError(f'{name} 需為 "x,y,z"，收到: {text}')
    return values


def create_node(context):
    data_path = LaunchConfiguration('data_path').perform(context)
    leaf = float(LaunchConfiguration('voxel_leaf_size').perform(context))
    input_file = LaunchConfiguration('input_file').perform(context) or os.path.join(data_path, 'map.pcd')
    output_file = LaunchConfiguration('output_file').perform(context) or \
        os.path.join(data_path, f'map_downsampled_{leaf:g}m.pcd')
    roi_min = parse_xyz(LaunchConfiguration('roi_min').perform(context), 'roi_min')
    roi_max = parse_xyz(LaunchConfiguration('roi_max').perform(context), 'roi_max')

    params = {
        'input_file': input_file,
        'output_file': output_file,
        'voxel_leaf_size': leaf,
        'min_points_per_voxel': int(LaunchConfiguration('min_points_per_voxel').perform(context)),
    }
    if roi_min or roi_max:
        params['roi_min'] = roi_min
        params['roi_max'] = roi_max
    return [Node(package='localization', executable='map_modified', name='map_modifier',
                 output='screen', parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('data_path', default_value='/root/data/Track3', description='Track 目錄'),
        DeclareLaunchArgument('input_file', default_value='', description='原始地圖；留空 = <data_path>/map.pcd'),
        DeclareLaunchArgument('output_file', default_value='', description='輸出；留空 = 依 leaf 自動命名'),
        DeclareLaunchArgument('voxel_leaf_size', default_value='1.0', description='體素大小 (m)，0 = 不降採樣'),
        DeclareLaunchArgument('min_points_per_voxel', default_value='1', description='少於此點數的體素視為雜訊剔除'),
        DeclareLaunchArgument('roi_min', default_value='', description='ROI 下界 "x,y,z"，留空 = 不裁切'),
        DeclareLaunchArgument('roi_max', default_value='', description='ROI 上界 "x,y,z"'),
        OpaqueFunction(function=create_node),
    ])
