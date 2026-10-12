#!/usr/bin/env python3
"""Measure ZED topic rates without deserializing messages (raw=True), so large images
don't bottleneck Python. Usage: measure_rates.py [seconds]   (run while zed_all.launch.py is up)"""
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Image, Imu, MagneticField, RelativeHumidity, Temperature

TOPICS = {
    '/image_combine_raw': Image,
    '/zed/imu/data_raw': Imu,
    '/zed/imu/mag': MagneticField,
    '/zed/atm_press': FluidPressure,
    '/zed/humidity': RelativeHumidity,
    '/zed/temperature/imu': Temperature,
    '/zed/temperature/left': Temperature,
}


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = rclpy.create_node('zed_measure_rates')
    stamps = {t: [] for t in TOPICS}
    for topic, msg_type in TOPICS.items():
        node.create_subscription(msg_type, topic, lambda _m, t=topic: stamps[t].append(time.time()),
                                 qos_profile_sensor_data, raw=True)
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    for topic, v in stamps.items():
        rate = (len(v) - 1) / (v[-1] - v[0]) if len(v) > 1 else 0.0
        print(f'{topic:26s} {rate:7.1f} Hz  (n={len(v)})')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
