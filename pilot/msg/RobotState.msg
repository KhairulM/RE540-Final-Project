# RobotState.msg
# Defines the current state of the robot

# State constants
uint8 IDLING = 1
uint8 EXPLORING = 2
uint8 PLANNING = 3
uint8 NAVIGATING = 4
uint8 SEARCHING = 5
uint8 GRASPING = 6

# Current state
uint8 state

# State name (for human readability)
string state_name
