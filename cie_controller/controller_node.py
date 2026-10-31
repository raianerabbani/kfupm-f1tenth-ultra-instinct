#!/usr/bin/env python3
"""
CIE-KFUPM F1TENTH Autonomous Racing Competition - Student Controller
Package: cie_controller
Node: controller_node
Executable: controller
Stage: 1 (Clean-lap qualification on Levine track)
"""

import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

try:
    from .centerline_data import LEVINE_CENTERLINE
except (ImportError, ValueError):
    try:
        from cie_controller.centerline_data import LEVINE_CENTERLINE
    except Exception:
        LEVINE_CENTERLINE = []


class F1TenthController(Node):
    def __init__(self):
        super().__init__('controller')

        # Declare configurable parameters
        self.declare_parameter('lookahead_distance', 1.25)
        self.declare_parameter('target_speed', 2.2)
        self.declare_parameter('min_speed', 1.3)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('steering_gain', 0.85)
        self.declare_parameter('max_steering_angle', 0.4189)  # ~24 deg
        self.declare_parameter('waypoints_file', '')

        self.lookahead = self.get_parameter('lookahead_distance').value
        self.target_speed = self.get_parameter('target_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.steering_gain = self.get_parameter('steering_gain').value
        self.max_steering = self.get_parameter('max_steering_angle').value

        # Load waypoints
        self.waypoints = self._load_waypoints()
        self.num_waypoints = len(self.waypoints)
        self.get_logger().info(f"Loaded {self.num_waypoints} waypoints for Levine track.")

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            qos_profile=10
        )

        # Publisher
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            qos_profile=10
        )

        # State tracking
        self.current_idx = 0
        self.emergency_stop = False
        self.min_forward_dist = float('inf')
        self.last_log_time = 0.0

        self.get_logger().info("CIE Controller initialized and ready.")

    def _load_waypoints(self):
        """Loads waypoints with multi-level fallbacks."""
        pts = []

        # 1. Check user-supplied parameter
        custom_path = self.get_parameter('waypoints_file').value
        if custom_path and os.path.isfile(custom_path):
            pts = self._read_csv(custom_path)
            if pts:
                return pts

        # 2. Check package share directory
        if get_package_share_directory is not None:
            try:
                pkg_share = get_package_share_directory('cie_controller')
                csv_path = os.path.join(pkg_share, 'racelines', 'levine_centerline.csv')
                if os.path.isfile(csv_path):
                    pts = self._read_csv(csv_path)
                    if pts:
                        return pts
            except Exception:
                pass

        # 3. Check local scratch repository path
        known_path = "/home/raian/Documents/scratch/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition/starter_ws/src/f1tenth_gym_ros/maps/levine_centerline.csv"
        if os.path.isfile(known_path):
            pts = self._read_csv(known_path)
            if pts:
                return pts

        # 4. Built-in Python data
        if LEVINE_CENTERLINE:
            return [(p[0], p[1]) for p in LEVINE_CENTERLINE]

        return pts

    def _read_csv(self, filepath):
        pts = []
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = [float(x.strip()) for x in line.split(',')[:2]]
                    pts.append((parts[0], parts[1]))
        except Exception as e:
            self.get_logger().warn(f"Failed to read CSV at {filepath}: {e}")
        return pts

    def scan_callback(self, scan_msg: LaserScan):
        """LiDAR safety check for emergency collision avoidance."""
        ranges = scan_msg.ranges
        num_ranges = len(ranges)
        if num_ranges == 0:
            return

        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment

        # Look in a forward cone of +/- 20 degrees
        cone_angle = math.radians(20)
        cone_min_idx = max(0, int((-cone_angle - angle_min) / angle_inc))
        cone_max_idx = min(num_ranges - 1, int((cone_angle - angle_min) / angle_inc))

        valid_forward = [
            r for r in ranges[cone_min_idx:cone_max_idx + 1]
            if not math.isnan(r) and not math.isinf(r) and scan_msg.range_min < r < scan_msg.range_max
        ]

        if valid_forward:
            self.min_forward_dist = min(valid_forward)
            # If an obstacle is critically close (< 0.35m), trigger safety brake
            self.emergency_stop = self.min_forward_dist < 0.35
        else:
            self.min_forward_dist = float('inf')
            self.emergency_stop = False

    def odom_callback(self, odom_msg: Odometry):
        if not self.waypoints:
            return

        # Vehicle position
        x_car = odom_msg.pose.pose.position.x
        y_car = odom_msg.pose.pose.position.y

        # Vehicle yaw from orientation quaternion
        q = odom_msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_car = math.atan2(siny_cosp, cosy_cosp)

        # 1. Find the closest waypoint to track progress along the circuit
        search_window = 50
        start_search = (self.current_idx - 10) % self.num_waypoints
        min_d_sq = float('inf')
        closest_idx = self.current_idx

        for i in range(search_window):
            idx = (start_search + i) % self.num_waypoints
            wx, wy = self.waypoints[idx]
            d_sq = (wx - x_car) ** 2 + (wy - y_car) ** 2
            if d_sq < min_d_sq:
                min_d_sq = d_sq
                closest_idx = idx

        self.current_idx = closest_idx

        # 2. Find lookahead target point along the loop
        target_idx = closest_idx
        cos_yaw = math.cos(yaw_car)
        sin_yaw = math.sin(yaw_car)

        for step in range(1, self.num_waypoints):
            idx = (closest_idx + step) % self.num_waypoints
            wx, wy = self.waypoints[idx]
            dx = wx - x_car
            dy = wy - y_car

            # Coordinates in car frame
            local_x = dx * cos_yaw + dy * sin_yaw
            local_y = -dx * sin_yaw + dy * cos_yaw

            # Must be ahead of car (local_x > 0)
            dist = math.sqrt(local_x * local_x + local_y * local_y)
            if dist >= self.lookahead and local_x > 0.05:
                target_idx = idx
                target_local_x = local_x
                target_local_y = local_y
                target_dist = dist
                break
        else:
            # Fallback if loop wasn't satisfied
            wx, wy = self.waypoints[(closest_idx + 15) % self.num_waypoints]
            dx = wx - x_car
            dy = wy - y_car
            target_local_x = dx * cos_yaw + dy * sin_yaw
            target_local_y = -dx * sin_yaw + dy * cos_yaw
            target_dist = math.sqrt(target_local_x ** 2 + target_local_y ** 2)

        # 3. Pure pursuit steering calculation
        # Curvature: kappa = 2 * y_local / (L^2)
        # Steering angle: delta = arctan(kappa * wheelbase) * gain
        curvature = 2.0 * target_local_y / (target_dist * target_dist)
        steering_angle = math.atan(curvature * self.wheelbase) * self.steering_gain

        # Clamp steering to physical limit
        steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))

        # 4. Dynamic speed profiling
        # Slow down on sharper turns for stability; speed up on straights
        steer_ratio = abs(steering_angle) / self.max_steering
        speed = self.target_speed - steer_ratio * (self.target_speed - self.min_speed)

        # LiDAR obstacle/proximity dampener
        if self.emergency_stop:
            speed = 0.0
            self.get_logger().warn(f"Emergency brake active! Forward dist: {self.min_forward_dist:.2f}m")
        elif self.min_forward_dist < 0.75:
            speed = min(speed, 1.0)

        # 5. Publish Ackermann drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'ego_racecar/base_link'
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)

        self.drive_pub.publish(drive_msg)

        # Periodic status logging
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_log_time > 2.0:
            self.last_log_time = now_sec
            self.get_logger().info(
                f"Pos: ({x_car:.2f}, {y_car:.2f}) | WP: {self.current_idx}/{self.num_waypoints} | "
                f"Steer: {math.degrees(steering_angle):.1f}° | Speed: {speed:.2f} m/s | Fwd: {self.min_forward_dist:.2f}m"
            )


def main(args=None):
    rclpy.init(args=args)
    controller = F1TenthController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero drive command on shutdown
        try:
            stop_msg = AckermannDriveStamped()
            stop_msg.header.stamp = controller.get_clock().now().to_msg()
            stop_msg.header.frame_id = 'ego_racecar/base_link'
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0
            controller.drive_pub.publish(stop_msg)
        except Exception:
            pass
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
