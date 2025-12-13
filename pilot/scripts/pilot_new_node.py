#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

from pilot.msg import RobotState
from pilot.srv import SetRobotState, SetRobotStateResponse

# Import state handlers
from state_handlers import (
    ExploringHandler,
)

class PilotNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("pilot_node", anonymous=False)
        
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.navigation_timeout = rospy.get_param("~navigation_timeout", 120.0)

        # Initialize state
        self.state = RobotState.IDLING
        
        # Create Publishers
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        
        # Initialize state handlers (after publishers are created)
        self.state_handlers = {
            RobotState.EXPLORING: ExploringHandler(self),
        }
        
        # Create Subscribers and Services
        self.state_subscriber = rospy.Subscriber(
            "/robot_state", RobotState, self.state_cb
        )
        self.set_state_client = rospy.ServiceProxy(
            "/set_robot_state", SetRobotState
        )
        rospy.loginfo("[Pilot] Pilot node initialized in state %s", self.state)
        
    def state_cb(self, msg):
        if self.state == msg.state:
            return # no state change, continue current behavior
        
        if msg.state == RobotState.EXPLORING:
            rospy.loginfo("[Pilot] Transitioning to EXPLORING state.")
            self.state = RobotState.EXPLORING
            self.state_handlers[RobotState.EXPLORING].run()
            self.set_state_client(RobotState.PLANNING)
        elif msg.state == RobotState.PLANNING:
            pass
        elif msg.state == RobotState.NAVIGATING:
            pass
        elif msg.state == RobotState.SEARCHING:
            pass
        elif msg.state == RobotState.GRASPING:
            pass
        elif msg.state == RobotState.IDLING:
            pass
        else:
            rospy.logwarn("[Pilot] Received unknown state: %d", msg.state)
            
    def run(self):
        rospy.spin()
            
            
def main():
    try:
        node = PilotNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Pilot] Node shutting down")
    except Exception as e:
        rospy.logerr("[Pilot] Fatal error: %s", str(e))


if __name__ == "__main__":
    main()