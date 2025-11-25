#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist
from chassis_control.msg import SetVelocity

class CmdVelToSetVelocity(object):
    def __init__(self):
        self.pub = rospy.Publisher(
            "/chassis_control/set_velocity",
            SetVelocity,
            queue_size=10
        )
        self.sub = rospy.Subscriber(
            "/cmd_vel",
            Twist,
            self.cb_cmd_vel,
            queue_size=10
        )
        rospy.loginfo("CmdVelToSetVelocity node started")

    def cb_cmd_vel(self, msg):
        vx = msg.linear.x   # m/s
        vy = msg.linear.y   # m/s
        wz = msg.angular.z  # rad/s

        vel_norm = math.sqrt(vx*vx + vy*vy) * 100.0   
        direction_deg = 90.0 + math.degrees(math.atan2(vy, vx))

        out = SetVelocity()
        out.velocity = vel_norm
        out.direction = direction_deg
        out.angular = wz

        self.pub.publish(out)

def main():
    rospy.init_node("cmd_vel_to_set_velocity")
    bridge = CmdVelToSetVelocity()
    rospy.spin()

if __name__ == "__main__":
    main()
