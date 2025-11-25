#!/usr/bin/python3
# coding=utf8
import rospy
import time 
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

ik = ik_transform.ArmIK()

joints_pub = None

current_position = {
    "x": 0.0,
    "y": 0.25,
    "z": 0.25
}

initial_position = {
    "x": 0.0,
    "y": 0.25,
    "z": 0.25
}

MIN_Z_DELTA = 0.01
start_manipulator = False
manipulator_finish = False

def robot_start_callback(msg):
    global robot_start
    robot_start = msg.data

def manipulator_command_callback(msg):
    global start_manipulator
    start_manipulator = msg.data

def move_to_initial_position():
    rospy.loginfo("Moving to initial position...")
    target = ik.setPitchRanges((current_position["x"], current_position["y"], current_position["z"]), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1000, ( 
            (1, 600), (2, 500), (3, servo_data['servo3']),
            (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])
        ))
        rospy.sleep(1)  
        rospy.loginfo("Moved to initial position.")
    else:
        rospy.logwarn("Failed to move to initial position.")

def stop():
    rospy.loginfo("Stopping and resetting to initial position.")
    target = ik.setPitchRanges((0.00, 0.25, 0.25), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1000, (  
            (1, 600), (2, 500), (3, servo_data['servo3']),
            (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])
        ))
        rospy.sleep(1)  

def target_position_callback(msg):
    global current_position, start_manipulator, initial_position
    

    if not start_manipulator:
        rospy.loginfo("Manipulator is not active. Ignoring target position updates.")
        stop()
        return
    
    dt_z = msg.z
    
    if abs(msg.z) < MIN_Z_DELTA:
        dt_z= 0
        manipulator_done_pub.publish(True)
        new_position = {
        "x": initial_position["x"] ,  
        "y": initial_position["y"] ,  
        "z": initial_position["z"] 
        }
        current_position["z"] = initial_position["z"]
        return
        
    new_position = {
        "x": current_position["x"] ,  
        "y": current_position["y"] ,  
        "z": current_position["z"] + dt_z * 0.5
    }

    rospy.loginfo(f"Updated target position: z={new_position['z']}")

    z = new_position["z"]
    if not (0.0 <= z <= 0.5): 
        rospy.logwarn("Target Z position is outside the robot's workspace.")
        return

    result = ik.setPitchRanges((new_position["x"], new_position["y"], new_position["z"]), -90, -180, 0)
    if result:
        servo_data = result[1]
        bus_servo_control.set_servos(joints_pub, 1000, (  
            (1, 600), (2, 500), (3, servo_data['servo3']),
            (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])
        ))
        time.sleep(1.5) 
        rospy.loginfo("Servos updated successfully.")

        current_position["z"] = new_position["z"]
    else:
        rospy.logwarn("No valid solution found for the given target Z position.")

    

if __name__ == '__main__':
    # ROS 노드 초기화
    rospy.init_node('kinematics_demo_z_axis', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    
    # Servo publisher 초기화
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    manipulator_done_pub = rospy.Publisher('/manipulator_done', Bool, queue_size=1)
    rospy.sleep(0.2)

    move_to_initial_position()

    # /target_position 토픽 구독
    rospy.Subscriber('/target_position', Point, target_position_callback)
    rospy.Subscriber('/manipulator_command', Bool, manipulator_command_callback) 

    rospy.loginfo("Waiting for Z-axis position updates...")
    
    rospy.spin()
