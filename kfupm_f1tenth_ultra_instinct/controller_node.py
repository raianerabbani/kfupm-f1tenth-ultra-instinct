#!/usr/bin/env python3
"""
CIE-KFUPM F1TENTH Autonomous Racing Competition
Team: Ultra Instinct
Package: kfupm_f1tenth_ultra_instinct
Simple, short, and safe Pure Pursuit controller for Stage 1.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

try:
    from .centerline_data import LEVINE_CENTERLINE
except (ImportError, ValueError):
    from kfupm_f1tenth_ultra_instinct.centerline_data import LEVINE_CENTERLINE


class SimpleSafeController(Node):
    def __init__(self):
        super().__init__('controller')
        self.waypoints = LEVINE_CENTERLINE
        self.lookahead = 1.1        # Lookahead distance (meters)
        self.speed_straight = 1.6   # Controlled speed on straights (m/s)
        self.speed_turn = 1.0       # Safe cornering speed (m/s)
        self.max_steer = 0.38       # Steering limit (~22 degrees)
        self.wheelbase = 0.33       # Vehicle wheelbase (meters)
        self.too_close = False

        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.get_logger().info("[Ultra Instinct] Simple & Safe Controller ready.")

    def scan_cb(self, scan: LaserScan):
        # LiDAR front safety cone (+/- 15 degrees)
        mid = len(scan.ranges) // 2
        cone = int(math.radians(15) / scan.angle_increment)
        front_ranges = [r for r in scan.ranges[mid - cone : mid + cone] if scan.range_min < r < scan.range_max]
        self.too_close = min(front_ranges) < 0.4 if front_ranges else False

    def odom_cb(self, odom: Odometry):
        # 1. Car pose and yaw
        xc = odom.pose.pose.position.x
        yc = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        # 2. Find closest waypoint globally across the entire circuit
        closest_idx = min(range(len(self.waypoints)), key=lambda i: (self.waypoints[i][0] - xc)**2 + (self.waypoints[i][1] - yc)**2)

        # 3. Look ahead along the circuit for target waypoint
        target_pt = self.waypoints[(closest_idx + 10) % len(self.waypoints)]
        for i in range(1, len(self.waypoints)):
            pt = self.waypoints[(closest_idx + i) % len(self.waypoints)]
            dx, dy = pt[0] - xc, pt[1] - yc
            lx = dx * cos_yaw + dy * sin_yaw
            ly = -dx * sin_yaw + dy * cos_yaw
            dist = math.hypot(lx, ly)
            if dist >= self.lookahead and lx > 0.1:
                target_pt = pt
                break

        # 4. Pure pursuit steering calculation
        dx, dy = target_pt[0] - xc, target_pt[1] - yc
        lx = dx * cos_yaw + dy * sin_yaw
        ly = -dx * sin_yaw + dy * cos_yaw
        dist = max(math.hypot(lx, ly), 0.1)

        # Ackermann steering: delta = atan(2 * L_wb * ly / dist^2)
        steer = math.atan2(2.0 * self.wheelbase * ly, dist * dist)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # 5. Smooth speed profiling (slows down on turns; stops if obstacle is too close)
        steer_intensity = abs(steer) / self.max_steer
        speed = self.speed_straight - steer_intensity * (self.speed_straight - self.speed_turn)
        if self.too_close:
            speed = 0.0

        # 6. Publish drive command
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_racecar/base_link'
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.drive_pub.publish(msg)


def main():
    rclpy.init()
    node = SimpleSafeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
