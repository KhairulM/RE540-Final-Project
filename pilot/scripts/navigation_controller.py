#!/usr/bin/env python3
"""
Navigation Controller Module
Handles robot base navigation and target tracking.
"""
import rospy
import math
from geometry_msgs.msg import Twist


class NavigationController:
    """Controls robot base movement to approach targets."""
    
    def __init__(self, cmd_pub, params):
        """
        Initialize the navigation controller.
        
        Args:
            cmd_pub: Publisher for velocity commands
            params: Dictionary of navigation parameters
        """
        self.cmd_pub = cmd_pub
        
        # Navigation parameters
        self.target_distance = params.get('target_distance', 0.20)
        self.distance_tolerance = params.get('distance_tolerance', 0.05)
        self.yaw_tolerance = params.get('yaw_tolerance', 0.1)
        self.max_linear_speed = params.get('max_linear_speed', 0.05)
        self.max_angular_speed = params.get('max_angular_speed', 0.3)
        self.Kp_distance = params.get('Kp_distance', 1.0)
        self.Kp_yaw = params.get('Kp_yaw', 0.0005)
        
        rospy.loginfo("[NavigationController] Initialized")
        rospy.loginfo("[NavigationController] Target distance: %.2f m", self.target_distance)
        rospy.loginfo("[NavigationController] Distance tolerance: %.2f m", self.distance_tolerance)
        rospy.loginfo("[NavigationController] Yaw tolerance: %.2f rad", self.yaw_tolerance)
    
    def compute_velocity_command(self, target_x, target_y):
        """
        Compute velocity command to approach target.
        
        Args:
            target_x, target_y: Target position in base_link frame
            
        Returns:
            tuple: (Twist message, bool reached_goal)
        """
        twist = Twist()
        
        # Calculate yaw error (angle to face the target)
        yaw_error = math.atan2(target_y, target_x)
        
        # Calculate distance in x-y plane
        current_distance = math.sqrt(target_x**2 + target_y**2)
        distance_error = target_x - self.target_distance
        
        # Check if we've reached the goal
        at_distance = abs(distance_error) <= self.distance_tolerance
        at_yaw = abs(yaw_error) <= self.yaw_tolerance
        reached_goal = at_distance and at_yaw
        
        if reached_goal:
            rospy.loginfo("[NavigationController] Goal reached! Distance: %.3f m, Yaw: %.3f rad", 
                         target_x, yaw_error)
            return twist, True
        
        # Yaw control (turn to face target)
        if abs(yaw_error) > self.yaw_tolerance:
            wz = -self.Kp_yaw * yaw_error
            
            # Clamp angular speed
            wz = max(min(wz, self.max_angular_speed), -self.max_angular_speed)
            
            twist.angular.z = wz
            rospy.loginfo("[NavigationController] Yaw error: %.3f rad, angular vel: %.3f", 
                         yaw_error, wz)
        
        # Distance control (forward/backward)
        if abs(distance_error) > self.distance_tolerance:
            v = self.Kp_distance * distance_error
            
            # Clamp linear speed
            v = max(min(v, self.max_linear_speed), -self.max_linear_speed)
            
            twist.linear.x = v
            rospy.loginfo("[NavigationController] Distance error: %.3f m, linear vel: %.3f", 
                         distance_error, v)
        
        return twist, False
    
    def stop(self):
        """Send stop command to robot."""
        self.cmd_pub.publish(Twist())
        rospy.loginfo("[NavigationController] Robot stopped")
    
    def publish_command(self, twist):
        """Publish velocity command."""
        self.cmd_pub.publish(twist)
