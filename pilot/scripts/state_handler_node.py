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
from enum import Enum
from std_msgs.msg import String

class State(Enum):
    IDLING = 1
    EXPLORING = 2
    PLANNING = 3
    NAVIGATING = 4
    SEARCHING = 5
    GRASPING = 6
    
class StateHandlerNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('state_handler_node', anonymous=False)
        
        # Initialize state
        self.current_state = State.IDLING
        
        # Publishers
        self.current_state_publisher = rospy.Publisher('/robot_state', String, queue_size=10)
        
        # Subscribers
        self.set_state_subscriber = rospy.Subscriber('/set_robot_state', String, self.set_state_callback)
        
        # Publish rate
        self.rate_hz = rospy.get_param("~rate", 10.0)
        self.rate = rospy.Rate(self.rate_hz)
        
        rospy.loginfo("[StateHandler] State handler node initialized in %s state", self.current_state.name)
        
    def publish_state(self):
        """Publish the current state."""
        state_str = self.current_state.name
        self.current_state_publisher.publish(state_str)
        
    def set_state_callback(self, msg):
        """Callback for setting new state via topic."""
        try:
            new_state_name = msg.data.upper()
            new_state = State[new_state_name]
            self.set_state(new_state)
        except KeyError:
            rospy.logwarn("[StateHandler] Invalid state name: %s", msg.data)
        
    def set_state(self, new_state):
        """Set a new state with validation and logging."""
        if not isinstance(new_state, State):
            rospy.logwarn("[StateHandler] Invalid state type")
            return False
            
        if self.current_state == new_state:
            rospy.logdebug("[StateHandler] Already in state: %s", new_state.name)
            return True
            
        # Validate state transition
        if self.is_valid_transition(self.current_state, new_state):
            old_state = self.current_state
            self.current_state = new_state
            rospy.loginfo("[StateHandler] State transition: %s -> %s", old_state.name, new_state.name)
            self.publish_state()
            return True
        else:
            rospy.logwarn("[StateHandler] Invalid state transition: %s -> %s", 
                         self.current_state.name, new_state.name)
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
            State.IDLING: [State.EXPLORING],
            State.EXPLORING: [State.PLANNING, State.IDLING],
            State.PLANNING: [State.NAVIGATING, State.IDLING],
            State.NAVIGATING: [State.SEARCHING, State.IDLING],
            State.SEARCHING: [State.GRASPING, State.NAVIGATING, State.IDLING],
            State.GRASPING: [State.SEARCHING, State.NAVIGATING, State.IDLING]
        }
        
        # Allow any state to transition to IDLING (emergency stop)
        if to_state == State.IDLING:
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
    
