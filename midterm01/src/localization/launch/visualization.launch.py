"""離線視覺化：重播 result.txt。

範例：
  ros2 launch localization visualization.launch.py data_path:=/root/data/Track1 \
      pose_file:=/root/midterm/src/localization/result/Track1_result.txt \
      reference_file:=/root/data/Track1/ref/gnss.csv accumulate_frames:=30 playback_rate:=2.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    data_path = arg('data_path')
    map_file = arg('map_file')
    if not map_file:
        # 優先使用降採樣快取；原始 map.pcd 為數 GB 的 ASCII，載入與顯示都很慢
        cache = os.path.join(data_path, 'map_downsampled_1m.pcd')
        map_file = cache if os.path.exists(cache) else os.path.join(data_path, 'map.pcd')
    track = os.path.basename(os.path.normpath(data_path))

    nodes = [Node(
        package='localization', executable='visualization', name='result_visualization', output='screen',
        parameters=[{
            'map_file': map_file,
            'pose_file': arg('pose_file') or f'/root/midterm/src/localization/result/{track}_result.txt',
            'ply_dir': arg('ply_dir') or os.path.join(data_path, 'raw_ply'),
            'reference_file': arg('reference_file'),
            'ply_scale': float(arg('ply_scale')),
            'frame_voxel': float(arg('frame_voxel')),
            'map_min_intensity': float(arg('map_min_intensity')),
            'map_crop_margin': float(arg('map_crop_margin')),
            'playback_rate': float(arg('playback_rate')),
            'start_frame': int(arg('start_frame')),
            'end_frame': int(arg('end_frame')),
            'frame_step': int(arg('frame_step')),
            'accumulate_frames': int(arg('accumulate_frames')),
            'loop': arg('loop').lower() == 'true',
            'keep_alive': True,
        }],
    )]
    if arg('rviz').lower() == 'true':
        rviz_config = os.path.join(get_package_share_directory('localization'), 'config', 'visualization.rviz')
        nodes.append(Node(package='rviz2', executable='rviz2', name='visualization_rviz', output='screen',
                          arguments=['-d', rviz_config]))
    return nodes


def generate_launch_description():
    args = [
        ('data_path', '/root/data/Track1', 'Track 目錄'),
        ('pose_file', '', '結果檔；留空 = result/<Track 目錄名>_result.txt'),
        ('ply_dir', '', 'PLY 目錄；留空 = <data_path>/raw_ply'),
        ('map_file', '', '地圖；留空 = 優先 map_downsampled_1m.pcd，否則 map.pcd'),
        ('reference_file', '', '參考軌跡 csv（含 x,y,z 欄位，如 ref/gnss.csv），僅供目視比對'),
        ('ply_scale', '8.0', 'PLY 尺度，需與定位設定一致'),
        ('frame_voxel', '0.0', '顯示用點雲降採樣 (m)，0 = 不降採樣'),
        ('map_min_intensity', '-1.0', '地圖 intensity 下限，< 0 = 不過濾'),
        ('map_crop_margin', '150.0', '只顯示軌跡外擴此距離內的地圖 (m)，< 0 = 全圖'),
        ('playback_rate', '1.0', '播放倍速，<= 0 = 不等待'),
        ('start_frame', '0', '起始幀'),
        ('end_frame', '-1', '結束幀（含），-1 = 到最後'),
        ('frame_step', '1', '每隔幾幀播放一次'),
        ('accumulate_frames', '0', '疊加最近 N 幀點雲於 /visual/accumulated，0 = 關閉'),
        ('loop', 'false', '播完是否重播'),
        ('rviz', 'true', '是否啟動 RViz'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args] +
        [OpaqueFunction(function=create_nodes)])
