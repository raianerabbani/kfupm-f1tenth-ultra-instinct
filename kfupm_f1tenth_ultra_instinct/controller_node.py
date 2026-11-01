#!/usr/bin/env python3
"""
CIE-KFUPM F1TENTH Autonomous Racing Competition
Team: Ultra Instinct
Package: kfupm_f1tenth_ultra_instinct
High-speed Pure Pursuit with aggressive early/longer braking.
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
        self.lookahead = 1.15       # Responsive steering lookahead (meters)
        self.speed_max = 2.2        # Maintained high straightaway speed (m/s)
        self.speed_turn = 0.9       # Safe corner apex speed (m/s)
        self.max_steer = 0.38       # Max steering angle (~22 degrees)
        self.wheelbase = 0.33       # Vehicle wheelbase (meters)
        self.min_forward_dist = float('inf')

        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.get_logger().info("[Ultra Instinct] 2.2 m/s Aggressive-Braking Controller ready.")

    def scan_cb(self, scan: LaserScan):
        # Forward LiDAR safety cone (+/- 18 degrees)
        mid = len(scan.ranges) // 2
        cone = int(math.radians(18) / scan.angle_increment)
        front = [r for r in scan.ranges[mid - cone : mid + cone] if scan.range_min < r < scan.range_max]
        self.min_forward_dist = min(front) if front else float('inf')

    def odom_cb(self, odom: Odometry):
        # 1. Car pose and orientation
        xc = odom.pose.pose.position.x
        yc = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        # 2. Global closest waypoint search across entire circuit
        closest_idx = min(range(len(self.waypoints)), key=lambda i: (self.waypoints[i][0] - xc)**2 + (self.waypoints[i][1] - yc)**2)

        # 3. Target lookahead point & upcoming preview point for early braking
        target_pt = self.waypoints[(closest_idx + 10) % len(self.waypoints)]
        preview_pt = self.waypoints[(closest_idx + 18) % len(self.waypoints)]  # ~2.2m ahead on track

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

        steer = math.atan2(2.0 * self.wheelbase * ly, dist * dist)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # 5. Lookahead anticipation: preview upcoming curvature for longer/earlier braking
        pdx, pdy = preview_pt[0] - xc, preview_pt[1] - yc
        ply = -pdx * sin_yaw + pdy * cos_yaw
        pdist = max(math.hypot(pdx, pdy), 0.1)
        preview_steer = abs(math.atan2(2.0 * self.wheelbase * ply, pdist * pdist))

        # 6. Aggressive braking profile (initiates early and brakes sharply)
        steer_factor = min(1.0, math.sqrt(max(abs(steer), preview_steer) / self.max_steer))
        speed = self.speed_max - steer_factor * (self.speed_max - self.speed_turn)

        # 7. Longer-distance LiDAR obstacle/wall safety damping
        if self.min_forward_dist < 0.45:
            speed = 0.0  # Emergency stop
        elif self.min_forward_dist < 1.1:
            speed = min(speed, 0.8)  # Aggressive deceleration when wall approaches head-on

        # 8. Publish drive command
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
