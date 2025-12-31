#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import time

class LapTimer(Node):
    def __init__(self):
        super().__init__('lap_timer')
        
        # 1. LISTEN to Position
        self.subscription = self.create_subscription(
            Odometry, 
            '/ego_racecar/odom', 
            self.odom_callback, 
            10)
            
        # 2. TALK to Simulator (To Teleport)
        self.teleport_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10)
            
        # State Variables
        self.start_pos = np.array([0.0, 0.0]) # Force start at Origin
        self.lap_start_time = time.time()
        self.current_lap = 1
        
        # Logic Flags
        self.left_start_zone = False  
        self.min_lap_time = 5.0       # Seconds
        self.start_radius = 1.0       # Meters

        # 3. TELEPORT ON STARTUP
        # We give it a small delay to make sure the Sim is listening
        time.sleep(1.0)
        self.teleport_to_start()
        
        self.get_logger().info("⏱️  LAP TIMER + TELEPORT STARTED!")

    def teleport_to_start(self):
        """ Sends the car back to (0,0) and resets timer """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Position (0,0)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        
        # Orientation (Facing forward, Quat: 0,0,0,1)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        self.teleport_pub.publish(msg)
        self.get_logger().info("✨ TELEPORTING TO START...")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        current_pos = np.array([x, y])
        dist_to_start = np.linalg.norm(current_pos - self.start_pos)
        
        # Check if we left the start zone
        if dist_to_start > 2.0:
            self.left_start_zone = True

        # Check for Lap Completion
        current_time = time.time()
        elapsed = current_time - self.lap_start_time

        if self.left_start_zone and (dist_to_start < self.start_radius) and (elapsed > self.min_lap_time):
            # ✅ LAP FINISHED
            self.print_lap_stats(elapsed)
            
            # 🔄 RESET PROCEDURE
            self.teleport_to_start()
            self.current_lap += 1
            self.lap_start_time = time.time()
            self.left_start_zone = False

    def print_lap_stats(self, lap_time):
        msg = f"""
        ========================================
        🏁 LAP {self.current_lap} COMPLETE!
        ⏱️  TIME: {lap_time:.4f} seconds
        ========================================
        """
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LapTimer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()