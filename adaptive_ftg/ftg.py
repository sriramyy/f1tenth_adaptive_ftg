#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String

from .parameters import Parameters
from .adaptive_logic import Logic

class AdaptiveFollowGap(Node):
    def __init__(self):
        super().__init__("adaptive_follow_gap")

        self.params = Parameters()
        self.logic = Logic()

        # ROS2 Setup
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        # state variables
        self.ego_speed = 0.0
        self.prev_steer = 0.0
        self.radians_per_elem = None
        self.proc_latest = None
        
        self.get_logger().info("✅ Adaptive FTG Started.")

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.ego_speed = float(np.hypot(vx, vy))

    def lidar_callback(self, scan):
        # 1. READ RAW DATA
        ranges = np.array(scan.ranges, dtype=np.float32)
        
        # determine the state with logic
        current_state = self.logic.detState(ranges)
        # print ranges for debugging
        print(f"[DEBUG] {ranges}")
        
        # update params based on the state
        self.params.setState(current_state)

        # 2. Preprocess (Uses Dynamic PREPROCESS_CONV_SIZE)
        proc = self.preprocess_lidar(ranges)
        self.proc_latest = proc.copy()

        if proc.size == 0: return

        # 3. Find Closest & Apply Safety Bubble
        closest_idx = int(np.argmin(proc))
        proc = self.mask_bubble(proc, closest_idx, self.params.BUBBLE_RADIUS)

        # 4. Find Max Gap & Goal Point
        gap_start, gap_end = self.find_max_gap(proc)
        best_idx = self.find_best_point(gap_start, gap_end, proc)

        # 5. Apply Handling Heuristics
        best_idx = self.apply_edge_guard(best_idx, proc.size)
        best_idx = self.apply_center_bias(best_idx, proc.size, self.params.CENTER_BIAS_ALPHA)

        # 6. Steering & Smoothing
        steer_raw = self.index_to_steer(best_idx, proc.size)
        steer_cmd = self.smooth_and_limit_steer(steer_raw)

        # 7. Speed Policy
        speed_cmd = self.speed_policy(steer_cmd)

        self.publish_drive(speed_cmd, steer_cmd)


    def preprocess_lidar(self, ranges):
        # sanitize invalid readings so zeros/inf do not create false obstacles that wobble the bubble
        ranges = np.nan_to_num(ranges, nan=self.params.MAX_LIDAR_DIST, posinf=self.params.MAX_LIDAR_DIST, neginf=0.0)
        ranges[ranges <= 0.0] = self.params.MAX_LIDAR_DIST

        n = len(ranges)
        self.radians_per_elem = (2.0 * np.pi) / n
        
        # Crop to front 180 degrees (approx)
        proc = ranges[135:-135].copy() if n > 270 else ranges.copy()

        # DYNAMIC BLUR: Straight = Blurry (Stable), Hairpin = Sharp (Precise)
        if self.params.PREPROCESS_CONV_SIZE > 1:
            k = np.ones(self.params.PREPROCESS_CONV_SIZE) / self.params.PREPROCESS_CONV_SIZE
            proc = np.convolve(proc, k, mode="same")

        # Clip using dynamic LOOKAHEAD
        return np.clip(proc, 0.0, self.params.MAX_LIDAR_DIST)

    def mask_bubble(self, arr, center, radius):
        a = arr.copy()
        lo = max(0, center - radius)
        hi = min(a.size, center + radius + 1)
        a[lo:hi] = 0.0
        return a

    def find_max_gap(self, arr):
        masked = np.ma.masked_where(arr == 0.0, arr)
        spans = np.ma.notmasked_contiguous(masked)
        if not spans: return 0, arr.size
        best = max(spans, key=lambda sl: (sl.stop - sl.start))
        return best.start, best.stop

    def find_best_point(self, start, stop, arr):
        if stop <= start + 1: return start
        seg = arr[start:stop]
        
        # We use a static conv size for finding the gap center to keep it robust
        if self.params.BEST_POINT_CONV_SIZE > 1:
            k = np.ones(self.params.BEST_POINT_CONV_SIZE) / self.params.BEST_POINT_CONV_SIZE
            seg = np.convolve(seg, k, mode="same")
        return int(np.argmax(seg)) + start

    def apply_edge_guard(self, idx, length):
        guard = int(round(np.deg2rad(self.params.EDGE_GUARD_DEG) / self.radians_per_elem))
        return int(np.clip(idx, guard, length - guard - 1))

    def apply_center_bias(self, idx, length, alpha):
        center = (length - 1) / 2.0
        return int(np.clip((1 - alpha) * idx + alpha * center, 0, length - 1))

    def index_to_steer(self, idx, length):
        angle = (idx - (length / 2.0)) * self.radians_per_elem
        # Clamp using DYNAMIC MAX_STEER (Tight in hairpins, limited in straights)
        return float(np.clip(angle / 1.8, -self.params.MAX_STEER_ABS, self.params.MAX_STEER_ABS))

    def smooth_and_limit_steer(self, steer):
        # DYNAMIC SMOOTHING: High Alpha = Snap turn, Low Alpha = Lazy turn
        s = (1.0 - self.params.STEER_SMOOTH_ALPHA) * steer + self.params.STEER_SMOOTH_ALPHA * self.prev_steer
        delta = np.clip(s - self.prev_steer, -self.params.STEER_RATE_LIMIT, self.params.STEER_RATE_LIMIT)
        self.prev_steer += delta
        return self.prev_steer

    def forward_clearance(self):
        if self.proc_latest is None: return self.params.MAX_LIDAR_DIST
        center = self.proc_latest.size // 2
        half = int(round(np.deg2rad(self.params.FWD_WEDGE_DEG) / self.radians_per_elem))
        return float(np.min(self.proc_latest[center-half : center+half+1]))

    def speed_policy(self, steer):
        # 1. Use the DYNAMIC SPEED set by the State Machine
        target_speed = self.params.CURRENT_SPEED
        
        # 2. Safety Override: TTC Braking
        # Even if the state says "Go Fast", if we are about to hit something, BRAKE.
        fwd = self.forward_clearance()
        ttc = fwd / max(self.ego_speed, 0.05)
        
        if ttc < self.params.TTC_HARD_BRAKE:
            target_speed = 0.0
        elif ttc < self.params.TTC_SOFT_BRAKE:
            scale = (ttc - self.params.TTC_HARD_BRAKE) / (self.params.TTC_SOFT_BRAKE - self.params.TTC_HARD_BRAKE)
            target_speed *= scale

        return target_speed

    def publish_drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

def main():
    rclpy.init()
    node = AdaptiveFollowGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n🛑 STOPPING CAR...")
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        node.drive_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()