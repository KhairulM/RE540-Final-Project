#!/usr/bin/env python3
"""
Navigation Controller Module
Handles robot base navigation and target tracking.
"""
import rospy
import math
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


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
        
        # Initialize move_base action client (lazy initialization)
        self.move_base_client = None
        
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
        
        # Distance control (forward/backward)
        if abs(distance_error) > self.distance_tolerance:
            v = self.Kp_distance * distance_error
            
            # Clamp linear speed
            v = max(min(v, self.max_linear_speed), -self.max_linear_speed)
            
            twist.linear.x = v
            rospy.loginfo("[NavigationController] Current distance: %.3f m, Distance error: %.3f m, linear vel: %.3f", 
                         current_distance, distance_error, v)
            
            # If distance error is negative (too close), only move backward without yaw control
            if distance_error < 0:
                return twist, False
        
        # Yaw control (turn to face target) - only when not backing up
        if abs(yaw_error) > self.yaw_tolerance:
            wz = self.Kp_yaw * yaw_error
            
            # Clamp angular speed
            wz = max(min(wz, self.max_angular_speed), -self.max_angular_speed)
            
            twist.angular.z = wz
            rospy.loginfo("[NavigationController] Yaw error: %.3f rad, angular vel: %.3f", 
                         yaw_error, wz)
        
        return twist, False
    
    def stop(self):
        """Send stop command to robot."""
        self.cmd_pub.publish(Twist())
        rospy.loginfo("[NavigationController] Robot stopped")
    
    def publish_command(self, twist):
        """Publish velocity command."""
        self.cmd_pub.publish(twist)
    
    def navigate_to_goal(self, x, y, yaw=0.0, frame_id="map", timeout=60.0):
        """
        Navigate to a goal position using move_base action server.
        
        Args:
            x: Target x position
            y: Target y position
            yaw: Target yaw orientation in radians (default: 0.0)
            frame_id: Reference frame for the goal (default: "map")
            timeout: Maximum time to wait for navigation (default: 60.0 seconds)
            
        Returns:
            bool: True if navigation succeeded, False otherwise
        """
        # Lazy initialization of move_base client
        if self.move_base_client is None:
            rospy.loginfo("[NavigationController] Initializing move_base action client...")
            self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            rospy.loginfo("[NavigationController] Waiting for move_base action server...")
            if not self.move_base_client.wait_for_server(rospy.Duration(5.0)):
                rospy.logerr("[NavigationController] move_base action server not available!")
                return False
            rospy.loginfo("[NavigationController] move_base action server connected")
        
        # Create goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set position
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        # Set orientation from yaw angle
        # Convert yaw to quaternion
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        rospy.loginfo("[NavigationController] Sending goal: (%.2f, %.2f, yaw=%.2f) in frame '%s'", 
                     x, y, yaw, frame_id)
        
        # Send goal
        self.move_base_client.send_goal(goal)
        
        # Wait for result with timeout
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(timeout))
        
        if not finished_within_time:
            rospy.logwarn("[NavigationController] Navigation timeout after %.1f seconds", timeout)
            self.move_base_client.cancel_goal()
            return False
        
        # Check result
        state = self.move_base_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[NavigationController] Successfully reached goal (%.2f, %.2f)", x, y)
            return True
        else:
            rospy.logwarn("[NavigationController] Navigation failed with state: %d", state)
            return False
