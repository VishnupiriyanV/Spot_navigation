#!/usr/bin/env python3

"""
Simple Robot Controller for Gazebo Spot Navigation
Controls a simple robot model through waypoints A->B->C->D in Gazebo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class GazeboRobotController(Node):
    def __init__(self):
        super().__init__('gazebo_robot_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Waypoints (A -> B -> C -> D)
        self.waypoints = [
            {'name': 'A', 'x': -3.0, 'y': 0.5},    # Red marker
            {'name': 'B', 'x': -1.0, 'y': -2.0},   # Green marker  
            {'name': 'C', 'x': 3.0, 'y': 0.5},     # Blue marker
            {'name': 'D', 'x': 1.0, 'y': 2.0}      # Yellow marker
        ]
        
        # Current position (estimated)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Navigation parameters
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        
        # Wait for Gazebo to initialize
        time.sleep(3)
        
        self.get_logger().info('🤖 Gazebo Robot Controller initialized')
        self.get_logger().info('🎯 Starting waypoint navigation: A -> B -> C -> D')
        
        # Start navigation
        self.navigate_waypoints()
    
    def send_velocity(self, linear_x, angular_z, duration, description=""):
        """Send velocity command for specified duration"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        if description:
            self.get_logger().info(f'{description}')
        
        # Send commands for the specified duration
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop robot
        self.stop_robot()
        time.sleep(0.5)
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def calculate_movement(self, target_x, target_y):
        """Calculate required movement to reach target"""
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        
        # Calculate distance and angle
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - self.current_yaw
        
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return distance, angle_diff
    
    def navigate_to_waypoint(self, waypoint):
        """Navigate to a specific waypoint"""
        target_x = waypoint['x']
        target_y = waypoint['y']
        name = waypoint['name']
        
        self.get_logger().info(f'🎯 Navigating to Point {name} ({target_x}, {target_y})')
        
        # Calculate required movement
        distance, angle_diff = self.calculate_movement(target_x, target_y)
        
        # Rotate towards target if needed
        if abs(angle_diff) > 0.1:  # 0.1 radians ≈ 5.7 degrees
            rotation_time = abs(angle_diff) / self.angular_speed
            rotation_direction = 1.0 if angle_diff > 0 else -1.0
            
            self.send_velocity(
                0.0, 
                rotation_direction * self.angular_speed, 
                rotation_time,
                f'🔄 Rotating towards Point {name}'
            )
            
            # Update current yaw
            self.current_yaw += angle_diff
        
        # Move forward to target
        if distance > 0.1:  # Only move if significant distance
            move_time = distance / self.linear_speed
            
            self.send_velocity(
                self.linear_speed,
                0.0,
                move_time, 
                f'➡️ Moving to Point {name} (distance: {distance:.2f}m)'
            )
            
            # Update current position
            self.current_x = target_x
            self.current_y = target_y
        
        self.get_logger().info(f'✅ Reached Point {name}!')
        time.sleep(1)
    
    def navigate_waypoints(self):
        """Navigate through all waypoints sequentially"""
        self.get_logger().info('🚀 Starting waypoint navigation sequence')
        
        for i, waypoint in enumerate(self.waypoints):
            self.get_logger().info(f'📍 Waypoint {i+1}/{len(self.waypoints)}')
            self.navigate_to_waypoint(waypoint)
        
        self.get_logger().info('🎉 Navigation complete! Successfully visited all waypoints A->B->C->D')
        
        # Final celebration movement
        self.get_logger().info('🎊 Performing victory spin!')
        self.send_velocity(0.0, 1.0, 2.0, '🌟 Victory spin!')
        
        self.get_logger().info('✨ Mission accomplished!')

def main(args=None):
    rclpy.init(args=args)
    
    controller = GazeboRobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Navigation interrupted by user')
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
