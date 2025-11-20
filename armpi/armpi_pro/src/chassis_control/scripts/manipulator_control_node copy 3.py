#!/usr/bin/env python3
# encoding:utf-8
# 2020/01/25 aiden
import time
import numpy as np
from math import sqrt
import rospy
from std_msgs.msg import Bool
import armpi_fpv.bus_servo_control as bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from kinematics import inverse_kinematics

# The robotic arm moves according to the angle calculated by inverse kinematics
ik = inverse_kinematics.IK()

class ArmIK:
    servo3Range = (0, 1000, 0, 240.0) # pulse width, angle
    servo4Range = (0, 1000, 0, 240.0)
    servo5Range = (0, 1000, 0, 240.0)
    servo6Range = (0, 1000, 0, 240.0)

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range, servo6_Range=servo6Range):
        # Adapt to different servos
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range
        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        # Convert the angle calculated by inverse kinematics into the pulse width value corresponding to the servo
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0])/2))
        servo4 = int(round(-theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0])/2))
        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0])/2 + theta5 * self.servo5Param))
        servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 + theta6)) * self.servo6Param)
        
        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}
    
    def setPitchRanges(self, coordinate_data, alpha, alpha1, alpha2, d = 0.01):
        # Given coordinate_data and pitch angle alpha, as well as the range of pitch angle range alpha1, alpha2, automatically find the solution closest to the given pitch angle
        # If there is no solution, return False, otherwise return the servo angle and pitch angle.
        # The coordinate unit is m, passed in as a tuple, for example (0, 0.5, 0.1)
        # alpha is a given pitch angle, in degrees
        # alpha1 and alpha2 are the range of pitch angle values
        x, y, z = coordinate_data
        a_range = abs(int(abs(alpha1 - alpha2)/d)) + 1
        for i in range(a_range):
            if i % 2:
                alpha_ = alpha + (i + 1)/2*d
            else:                
                alpha_ = alpha - i/2*d
                if alpha_ < alpha1:
                    alpha_ = alpha2 - i/2*d
            result = ik.getRotationAngle((x, y, z), alpha_)
            if result:
                theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                return result, servos, alpha_
        
        return False

class ArmIKNode:
    def __init__(self):
        rospy.init_node('ik_test', log_level=rospy.DEBUG)
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

        # self.grab_done_sub = rospy.Subscriber('/grab_done', Bool, self.grab_done_callback)
        self.grab_done_pub = rospy.Publisher('/grab_done', Bool, queue_size=1)
        self.grab_done_sub = rospy.Subscriber('/move_done', Bool, self.move_done_callback)
        self.AK = ArmIK()
        self.grab_active = False
        rospy.sleep(0.2)

    # def grab_done_callback(self, msg):
    #     if not msg.data and not self.grab_active:
    #         self.grab_active = True
    #         self.execute_motion()

    def move_done_callback(self, msg):
        if not msg.data and not self.grab_active:
            self.grab_active = True
            self.execute_motion()

    def execute_motion(self):
        if rospy.is_shutdown():
            return

        target_pitch_3 = -145
        target = self.AK.setPitchRanges((0.0, 0.28, -0.07), target_pitch_3, target_pitch_3, target_pitch_3)
        if target:
            servo_data = target[1]
            print("Servo3: {}, Servo4: {}, Servo5: {}, Servo6: {}".format(servo_data['servo3'], servo_data['servo4'], servo_data['servo5'], servo_data['servo6']))
            
            bus_servo_control.set_servos(self.joints_pub, 1000, ((1, 250), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            time.sleep(1)
            bus_servo_control.set_servos(self.joints_pub, 1000, ((1, 500), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
        time.sleep(5)

        target = self.AK.setPitchRanges((0.0, 0.25, 0.2), -90, -90, 0)
        if target:
            servo_data = target[1]
            print("Servo3: {}, Servo4: {}, Servo5: {}, Servo6: {}".format(servo_data['servo3'], servo_data['servo4'], servo_data['servo5'], servo_data['servo6']))
            
            bus_servo_control.set_servos(self.joints_pub, 1000, ((1, 500), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            time.sleep(3)
            bus_servo_control.set_servos(self.joints_pub, 1000, ((1, 250), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
        time.sleep(2)

        # Publish motion done
        self.grab_done_pub.publish(True)
        self.grab_active = False

if __name__ == "__main__":
    arm_ik_node = ArmIKNode()
    rospy.spin()
