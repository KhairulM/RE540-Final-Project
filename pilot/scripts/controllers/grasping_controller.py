#!/usr/bin/env python3
"""
Grasping Controller Module
Handles all arm manipulation and grasping sequences.
"""
import rospy
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from kinematics import ik_transform
from armpi_pro import bus_servo_control


class GraspingController:
    """Controls the robotic arm for grasping operations."""
    
    def __init__(self, joints_pub):
        """
        Initialize the grasping controller.
        
        Args:
            joints_pub: Publisher for servo commands
        """
        self.joints_pub = joints_pub
        self.inverse_kinematics = ik_transform.ArmIK()
        self.init_pose = (0.0, 0.18, 0.13)
        self.is_initialized = False
        
        rospy.loginfo("[GraspingController] Initialized")
    
    def initialize_arm(self):
        """Move arm to home/init position."""
        rospy.loginfo("[GraspingController] Moving to home position...")
        target = self.inverse_kinematics.setPitchRanges(self.init_pose, -90, -180, 0)
        
        if target: 
            servo_data = target[1]
            bus_servo_control.set_servos(
                self.joints_pub, 1500, 
                ((1, 50), (2, 500), (3, servo_data['servo3']),
                 (4, servo_data['servo4']), (5, servo_data['servo5']), 
                 (6, servo_data['servo6']))
            )
            rospy.sleep(2.0)
            self.is_initialized = True
            rospy.loginfo("[GraspingController] Arm initialized at home position")
            return True
        else:
            rospy.logwarn("[GraspingController] Failed to compute IK for init position")
            return False
    
    def approach_target(self, x, y, z):
        """
        Move arm to approach position above the target.
        
        Args:
            x, y, z: Target position in base_link frame
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("[GraspingController] Approaching target at x=%.3f, y=%.3f, z=%.3f", x, y, z)
        
        # Swap coordinates for arm IK frame convention
        arm_x = y
        arm_y = x
        arm_z = z
        
        target = self.inverse_kinematics.setPitchRanges(
            (arm_x, arm_y, arm_z), 
            -145, -180, 0
        )
        
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(
                self.joints_pub, 1500, 
                ((1, 50), (2, 500), (3, servo_data['servo3']),
                 (4, servo_data['servo4']), (5, servo_data['servo5']), 
                 (6, servo_data['servo6']))
            )
            rospy.sleep(2.0)
            return True
        else:
            rospy.logwarn("[GraspingController] Failed to compute IK for approach position")
            return False
    
    def grasp_object(self, x, y, z):
        """
        Close gripper at target position to grasp object.
        
        Args:
            x, y, z: Target position in base_link frame
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("[GraspingController] Grasping object...")
        
        # Swap coordinates for arm IK frame convention
        arm_x = y
        arm_y = x
        arm_z = z
        
        target = self.inverse_kinematics.setPitchRanges(
            (arm_x, arm_y, arm_z), 
            -145, -180, 0
        )
        
        if target:
            servo_data = target[1]
            # Close gripper with higher value for tighter grip
            bus_servo_control.set_servos(
                self.joints_pub, 3000, 
                ((1, 600), (2, 500), (3, servo_data['servo3']),
                 (4, servo_data['servo4']), (5, servo_data['servo5']), 
                 (6, servo_data['servo6']))
            )
            rospy.sleep(3.5)
            rospy.loginfo("[GraspingController] Object grasped")
            return True
        else:
            rospy.logwarn("[GraspingController] Failed to compute IK for grasp position")
            return False
    
    def move_to_basket(self):
        """Move to basket position while holding object."""
        rospy.loginfo("[GraspingController] Moving to basket...")
        bus_servo_control.set_servos(
            self.joints_pub, 1500, 
            ((1, 700), (2, 500), (3, 900), (4, 365), (5, 460), (6, 500))
        )
        rospy.sleep(2.0)
    
    def place_object(self):
        """Open gripper to release object in basket."""
        rospy.loginfo("[GraspingController] Placing object...")
        bus_servo_control.set_servos(
            self.joints_pub, 1500, 
            ((1, 200), (2, 500), (3, 900), (4, 365), (5, 460), (6, 500))
        )
        rospy.sleep(2.0)
        rospy.loginfo("[GraspingController] Object placed")
    
    def execute_full_sequence(self, target_x, target_y, target_z):
        """
        Execute complete pick and place sequence.
        
        Args:
            target_x, target_y, target_z: Target position in base_link frame
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("[GraspingController] Starting full grasp sequence...")
        
        # Approach
        if not self.approach_target(target_x, target_y, target_z):
            return False
        
        # Grasp
        if not self.grasp_object(target_x, target_y, target_z):
            return False
        
        # Move to basket
        self.move_to_basket()
        
        # Place
        self.place_object()
        
        # Return home
        self.initialize_arm()
        
        rospy.loginfo("[GraspingController] Grasp sequence complete!")
        return True
