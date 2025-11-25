#!/usr/bin/python3
# coding=utf8
import sys
import time
import rospy
import rospkg

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, Point
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from kinematics import ik_transform
from armpi_pro import bus_servo_control

class PickandPlace():
    def __init__(self):
        rospy.init_node('pick_and_place_node')
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.finish_sign = rospy.Publisher('/grab_finish', Bool, queue_size=10, latch=True)
        # self.move_base_pub = rospy.Publisher('/move_base', Point, queue_size=1)
        
        self.inverse_kinematics = ik_transform.ArmIK()
        self.target_pose = PointStamped()
        self.move_base_pose = Point()
        self.init_pose = (0.0, 0.18, 0.13)
        self.mode = "Idle"  # States: "Idle", "Grasping", "Placing"
        self.move_finished = True
        self.has_target = False
        self.is_initialized = False
        self.is_busy = False
        
        # Subscribers
        rospy.Subscriber('/target_pose', PointStamped, self.target_callback, queue_size=1)
        rospy.Subscriber('/move_finish', Bool, self.finish_callback, queue_size=1)
        rospy.Subscriber('/start_grasp', Bool, self.start_grasp_callback, queue_size=1)
        
        rospy.loginfo("[PickandPlace] Node initialized. Waiting for commands...")
        
    def target_callback(self, msg):
        """Receives target pose for grasping."""
        self.target_pose = msg
        self.has_target = True
        # rospy.loginfo("[PickandPlace] Received target pose: x=%.3f, y=%.3f, z=%.3f", msg.point.x, msg.point.y, msg.point.z)
        
    def start_grasp_callback(self, msg):
        """Trigger to start grasping sequence."""
        if msg.data and self.has_target and not self.is_busy:
            self.mode = "Grasping"
            rospy.loginfo("[PickandPlace] Starting grasp sequence... %.3f, %.3f, %.3f", 
                          self.target_pose.point.x, 
                          self.target_pose.point.y, 
                          self.target_pose.point.z)
        
    def finish_callback(self, msg):
        """Callback function for /move_finish topic."""
        self.move_finished = msg.data
        if msg.data:
            rospy.loginfo("[PickandPlace] Base movement finished")


    def arm_planning(self):
        """Execute full pick and place sequence."""
        rospy.loginfo("[PickandPlace] Moving to approach position...")
        
        # Adjust coordinates for arm IK frame (swap or negate if needed)
        # Try swapping x and y if the arm frame expects different convention
        arm_x = self.target_pose.point.y
        arm_y = self.target_pose.point.x
        arm_z = self.target_pose.point.z
        
        rospy.loginfo("[PickandPlace] Target in base_link: x=%.3f, y=%.3f, z=%.3f", arm_x, arm_y, arm_z)
        
        # Move to approach position (above target)
        target = self.inverse_kinematics.setPitchRanges(
            (arm_x, arm_y, arm_z), 
            -145, -180, 0)
        
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(self.joints_pub, 1500, 
                ((1, 50), (2, 500), (3, servo_data['servo3']),
                 (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(1.5)
        else:
            rospy.logwarn("[PickandPlace] Failed to compute IK for approach position")
            return False
        
        # Grasp the object
        rospy.loginfo("[PickandPlace] Grasping object...")
        self.grasping(arm_x, 
                     arm_y, 
                     arm_z)
        
        # Lift to basket position
        rospy.loginfo("[PickandPlace] Moving to basket...")
        self.basket_planning()
        
        # Place in basket
        rospy.loginfo("[PickandPlace] Placing object...")
        self.placing()
        
        # Signal completion
        rospy.loginfo("[PickandPlace] Grasp sequence complete!")
        
        # Return to init
        rospy.loginfo("[PickandPlace] Returning to home position...")
        self.init()
        
        rospy.sleep(1.0)
        self.finish_sign.publish(True)
        
        return True
    
    def init(self):
        """Move arm to home/init position."""
        target = self.inverse_kinematics.setPitchRanges(self.init_pose, -90, -180, 0)
        if target: 
            servo_data = target[1]
            bus_servo_control.set_servos(self.joints_pub, 1500, 
                ((1, 50), (2, 500), (3, servo_data['servo3']),
                 (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(1.5)
            self.is_initialized = True
            return True
        else:
            rospy.logwarn("[PickandPlace] Failed to compute IK for init position")
            return False
    
    def grasping(self, target_x, target_y, target_z):
        """Close gripper at target position."""
        target = self.inverse_kinematics.setPitchRanges((target_x, target_y, target_z), -145, -180, 0)
        if target:
            servo_data = target[1]
            # Close gripper with higher value for tighter grip
            bus_servo_control.set_servos(self.joints_pub, 3000, 
                ((1, 550), (2, 500), (3, servo_data['servo3']),
                 (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(3.5)
            return True
        else:
            rospy.logwarn("[PickandPlace] Failed to compute IK for grasp position")
            return False
        
    def basket_planning(self):
        """Move to basket position while holding object."""
        bus_servo_control.set_servos(self.joints_pub, 1500, 
            ((1, 700), (2, 500), (3, 900), (4, 365), (5, 460), (6, 500)))
        rospy.sleep(1.5)
        
    def placing(self):
        """Open gripper to release object in basket."""
        bus_servo_control.set_servos(self.joints_pub, 1500, 
            ((1, 200), (2, 500), (3, 900), (4, 365), (5, 460), (6, 500)))
        rospy.sleep(1.5)
        
    def run(self):
        """Main control loop."""
        rate = rospy.Rate(10)
        
        # Initialize arm on startup
        rospy.loginfo("[PickandPlace] Initializing arm position...")
        self.init()
        
        while not rospy.is_shutdown():
            if self.mode == "Grasping" and not self.is_busy:
                self.is_busy = True
                
                # Execute pick and place sequence
                success = self.arm_planning()
                
                # Reset state
                self.mode = "Idle"
                self.has_target = False
                self.is_busy = False
                
                if success:
                    rospy.loginfo("[PickandPlace] Ready for next command")
                else:
                    rospy.logwarn("[PickandPlace] Grasp sequence failed")
            
            rate.sleep()

def main():
    try:
        rospy.loginfo("********RUNNING PICK & PLACE NODE************")
        p_p = PickandPlace()
        p_p.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[PickandPlace] Node shutdown")
    except Exception as e:
        rospy.logerr(f"[PickandPlace] Error: {e}")

# Main
if __name__ == '__main__':
    main()
