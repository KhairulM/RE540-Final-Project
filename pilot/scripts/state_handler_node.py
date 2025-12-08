# States:
# 1. Idling (Nothing to do)
# 2. Exploring (Go around the environment, build a semantic map)
# 3. Planning (List of all actions to take)
# 4. Navigating (Go to the store)
# 5. Searching (Look for the object on the store front)
# 6. Grasping (Pick up the object and put it into the basket)

# States transition:
# Idling -> Exploring (When a new task is assigned)
# Exploring -> Planning (When the semantic map is built)
# Planning -> Navigating (When the plan is ready)
# Navigating -> Searching (When the robot reaches the store)
# Searching -> Grasping (When the object is found)
# Grasping -> Searching (Searching for the object to pick, until no more objects to pick up)
# Searching -> Navigating (Go to the next store, if any. If no more stores, go to the drop-off point)

import rospy
from pilot.msg import PilotState
from pilot.srv import SetPilotState, SetPilotStateResponse
    
class StateHandlerNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('state_handler_node', anonymous=False)
        
        # Initialize state
        self.current_state = PilotState.IDLING
        
        # State name mapping
        self.state_names = {
            PilotState.IDLING: "IDLING",
            PilotState.EXPLORING: "EXPLORING",
            PilotState.PLANNING: "PLANNING",
            PilotState.NAVIGATING: "NAVIGATING",
            PilotState.SEARCHING: "SEARCHING",
            PilotState.GRASPING: "GRASPING"
        }
        
        # Publishers
        self.current_state_publisher = rospy.Publisher('/robot_state', PilotState, queue_size=10)
        
        # Services
        self.set_state_service = rospy.Service('/set_robot_state', SetPilotState, self.set_state_callback)
        
        # Publish rate
        self.rate_hz = rospy.get_param("~rate", 10.0)
        self.rate = rospy.Rate(self.rate_hz)
        
        rospy.loginfo("[StateHandler] State handler node initialized in %s state", 
                     self.state_names[self.current_state])
        
    def publish_state(self):
        """Publish the current state."""
        msg = PilotState()
        msg.state = self.current_state
        msg.state_name = self.state_names[self.current_state]
        self.current_state_publisher.publish(msg)
        
    def set_state_callback(self, req):
        """Service callback for setting new state."""
        response = SetPilotStateResponse()
        
        # Validate state value
        if req.state not in self.state_names:
            response.success = False
            response.message = "Invalid state value: {}".format(req.state)
            rospy.logwarn("[StateHandler] %s", response.message)
            return response
        
        # Attempt to set state
        success = self.set_state(req.state)
        
        if success:
            response.success = True
            response.message = "State changed to {}".format(self.state_names[req.state])
        else:
            response.success = False
            response.message = "Invalid state transition: {} -> {}".format(
                self.state_names[self.current_state], 
                self.state_names[req.state]
            )
        
        return response
        
    def set_state(self, new_state):
        """Set a new state with validation and logging."""
        if new_state not in self.state_names:
            rospy.logwarn("[StateHandler] Invalid state value: %d", new_state)
            return False
            
        if self.current_state == new_state:
            rospy.logdebug("[StateHandler] Already in state: %s", self.state_names[new_state])
            return True
            
        # Validate state transition
        if self.is_valid_transition(self.current_state, new_state):
            old_state = self.current_state
            self.current_state = new_state
            rospy.loginfo("[StateHandler] State transition: %s -> %s", 
                         self.state_names[old_state], self.state_names[new_state])
            self.publish_state()
            return True
        else:
            rospy.logwarn("[StateHandler] Invalid state transition: %s -> %s", 
                         self.state_names[self.current_state], self.state_names[new_state])
            return False
    
    def is_valid_transition(self, from_state, to_state):
        """
        Check if a state transition is valid based on the state machine rules.
        
        Valid transitions:
        - IDLING -> EXPLORING
        - EXPLORING -> PLANNING
        - PLANNING -> NAVIGATING
        - NAVIGATING -> SEARCHING
        - SEARCHING -> GRASPING
        - GRASPING -> SEARCHING (continue searching for more objects)
        - SEARCHING -> NAVIGATING (move to next store or drop-off)
        - Any state -> IDLING (emergency stop/reset)
        """
        valid_transitions = {
            PilotState.IDLING: [PilotState.EXPLORING],
            PilotState.EXPLORING: [PilotState.PLANNING, PilotState.IDLING],
            PilotState.PLANNING: [PilotState.NAVIGATING, PilotState.IDLING],
            PilotState.NAVIGATING: [PilotState.SEARCHING, PilotState.IDLING],
            PilotState.SEARCHING: [PilotState.GRASPING, PilotState.NAVIGATING, PilotState.IDLING],
            PilotState.GRASPING: [PilotState.SEARCHING, PilotState.NAVIGATING, PilotState.IDLING]
        }
        
        # Allow any state to transition to IDLING (emergency stop)
        if to_state == PilotState.IDLING:
            return True
            
        return to_state in valid_transitions.get(from_state, [])
    
    def spin(self):
        """Main loop for the state handler node."""
        rospy.loginfo("[StateHandler] Starting state handler loop...")
        
        # Publish initial state
        self.publish_state()
        
        while not rospy.is_shutdown():
            # Continuously publish current state
            self.publish_state()
            
            # Sleep to maintain loop rate
            self.rate.sleep()


def main():
    try:
        node = StateHandlerNode()
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[StateHandler] State handler node shutting down")


if __name__ == "__main__":
    main()
    
