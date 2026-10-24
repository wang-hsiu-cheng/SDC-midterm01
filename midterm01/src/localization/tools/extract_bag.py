#!/usr/bin/env python3
"""從 TrackN.db3 匯出 IMU 與 GNSS（僅作離線評估參考）為 csv。

用法（container 內，需先 source /opt/ros/jazzy/setup.bash）：
  python3 extract_bag.py /root/data/Track1/Track1.db3 /root/data/Track1/ref
輸出：<out_dir>/imu.csv、<out_dir>/gnss.csv（時間戳一律用 header.stamp，單位 ns）
"""
import argparse
import csv
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

TOPICS = {
    '/imu/data': Imu,
    '/localizer_node/gnss_pose': PoseStamped,
}


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', help='.db3 檔案路徑')
    parser.add_argument('out_dir')
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))

    with open(os.path.join(args.out_dir, 'imu.csv'), 'w', newline='') as imu_f, \
         open(os.path.join(args.out_dir, 'gnss.csv'), 'w', newline='') as gnss_f:
        imu_w = csv.writer(imu_f)
        gnss_w = csv.writer(gnss_f)
        imu_w.writerow(['stamp_ns', 'bag_ns', 'qx', 'qy', 'qz', 'qw', 'wx', 'wy', 'wz', 'ax', 'ay', 'az'])
        gnss_w.writerow(['stamp_ns', 'bag_ns', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        counts = {t: 0 for t in TOPICS}
        while reader.has_next():
            topic, data, bag_ns = reader.read_next()
            msg = deserialize_message(data, TOPICS[topic])
            counts[topic] += 1
            if topic == '/imu/data':
                o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
                imu_w.writerow([stamp_ns(msg.header), bag_ns, o.x, o.y, o.z, o.w, w.x, w.y, w.z, a.x, a.y, a.z])
            else:
                p, q = msg.pose.position, msg.pose.orientation
                gnss_w.writerow([stamp_ns(msg.header), bag_ns, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    print(counts)


if __name__ == '__main__':
    main()
