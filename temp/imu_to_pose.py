#!/usr/bin/env python3
"""
imu_to_pose.py (ROS 2)

Subscribes:  sensor_msgs/msg/Imu      (e.g. /imu/filtered)
Publishes:   geometry_msgs/msg/PoseStamped (e.g. /imu/pose)

Purpose:
RViz2 Jazzy doesn't include an IMU display. This node converts IMU orientation
(quaternion) into a PoseStamped so RViz can visualize it with the Pose display.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped


class ImuToPose(Node):
    def __init__(self):
        super().__init__('imu_to_pose')

        # Parameters
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('pose_topic', '/imu/pose')
        self.declare_parameter('pose_frame_id', '')   # if empty => use IMU message frame_id
        self.declare_parameter('fixed_position', [0.0, 0.0, 0.0])  # x,y,z

        imu_topic = self.get_parameter('imu_topic').value
        pose_topic = self.get_parameter('pose_topic').value

        self._pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self._imu_sub = self.create_subscription(Imu, imu_topic, self.imu_cb, 10)

        self.get_logger().info(f"Subscribing IMU: {imu_topic}")
        self.get_logger().info(f"Publishing Pose: {pose_topic}")

    def imu_cb(self, msg: Imu):
        pose_msg = PoseStamped()

        # stamp
        pose_msg.header.stamp = msg.header.stamp

        # frame_id: use parameter override if set, else keep IMU frame_id
        frame_override = self.get_parameter('pose_frame_id').value
        pose_msg.header.frame_id = frame_override if frame_override else msg.header.frame_id

        # fixed position (so RViz has something to draw)
        pos = self.get_parameter('fixed_position').value
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])

        # orientation from IMU
        pose_msg.pose.orientation = msg.orientation

        self._pose_pub.publish(pose_msg)


def main():
    rclpy.init()
    node = ImuToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
