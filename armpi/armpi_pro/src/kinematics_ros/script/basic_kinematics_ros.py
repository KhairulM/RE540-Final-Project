#!/usr/bin/python3
# coding=utf8
import rospy
import time
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from std_msgs.msg import Bool

ik = ik_transform.ArmIK()

# Global variable to track /move_finish state
move_finish = False

def move_finish_callback(msg):
    """Callback function for /move_finish topic."""
    global move_finish
    move_finish = msg.data

def publish_servos(joints_pub, duration, servo_values):
    """Helper function to publish servo values."""
    bus_servo_control.set_servos(joints_pub, duration, servo_values)

def execute_motion_sequence(targets, joints_pub, grab_finish_pub):
    """Execute motion sequence for given targets."""
    for target in targets:
        coordinate, alpha, alpha1, alpha2, servo_adjustments = (
            target["coordinate"], target["alpha"], target["alpha1"],
            target["alpha2"], target["servos"]
        )
        result = ik.setPitchRanges(coordinate, alpha, alpha1, alpha2)
        if result:
            servo_data = result[1]
            publish_servos(
                joints_pub, 1500, (
                    (1, servo_adjustments[0]),
                    (2, servo_adjustments[1]),
                    (3, servo_data['servo3']),
                    (4, servo_data['servo4']),
                    (5, servo_data['servo5']),
                    (6, servo_data['servo6']),
                )
            )
        else:
            rospy.logwarn(f"No valid IK solution for {coordinate}.")
        time.sleep(1.5)  # Wait before the next motion

    # Publish /grab_finish after completing all motions
    grab_finish_pub.publish(True)
    rospy.loginfo("Published /grab_finish: True")

def stop(joints_pub):
    """Stop function to reset servos to initial position."""
    target = ik.setPitchRanges((0.00, 0.2, 0.2), -90, -180, 0)
    if target:
        servo_data = target[1]
        publish_servos(
            joints_pub, 1500, (
                (1, 200), (2, 500),
                (3, servo_data['servo3']),
                (4, servo_data['servo4']),
                (5, servo_data['servo5']),
                (6, servo_data['servo6']),
            )
        )

if __name__ == '__main__':
    rospy.init_node('kinematics_demo', log_level=rospy.DEBUG)
    rospy.on_shutdown(lambda: stop(joints_pub))

    # Publishers and subscribers
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    grab_finish_pub = rospy.Publisher('/grab_finish', Bool, queue_size=1)
    rospy.Subscriber('/move_finish', Bool, move_finish_callback)
    
    rospy.sleep(0.2)  # Allow some time for connections

    # Define motion targets
    targets = [
        # {"coordinate": (0.0, 0.30, -0.06), "alpha": -140, "alpha1": -180, "alpha2": 0, "servos": [200, 500]},
        # {"coordinate": (0.0, 0.30, -0.06), "alpha": -140, "alpha1": -180, "alpha2": 0, "servos": [500, 500]},
        
        {"coordinate": (0.0, 0.2, 0.2), "alpha": 90, "alpha1": 180, "alpha2": 0, "servos": [500, 500]},
        # {"coordinate": (0.0, 0.2, 0.25), "alpha": 30, "alpha1": 180, "alpha2": 0, "servos": [500, 500]},
        
        
        # {"coordinate": (0.0, 0.0, 0.25), "alpha": 0, "alpha1": -180, "alpha2": 0, "servos": [500, 500]},
        
        # {"coordinate": (0.0, 0.2, 0.2), "alpha": -90, "alpha1": -180, "alpha2": 0, "servos": [500, 500]},
        # {"coordinate": (0.0, 0.2, 0.2), "alpha": -90, "alpha1": -180, "alpha2": 0, "servos": [200, 500]},
        # {"coordinate": (0.0,-0.2, 0.2), "alpha": 90, "alpha1": 0, "alpha2": 90, "servos": [200, 500]},
    ]

    rospy.loginfo("Waiting for /move_finish to become True...")
    while not rospy.is_shutdown():
        if move_finish:
            rospy.loginfo("Starting motion sequence...")
            execute_motion_sequence(targets, joints_pub, grab_finish_pub)
            move_finish = False  # Reset for next cycle
        else:
            rospy.sleep(0.1)  # Poll at a short interval
