#!/usr/bin/env python3
import rospy
import actionlib
import argparse
from actionlib import GoalStatus
from hiwonder_servo_msgs.msg import ActionGroupRunnerAction, ActionGroupRunnerGoal


def parse_args():
    """
    Parse command-line arguments using ROS-friendly argv (rospy.myargv).
    """
    parser = argparse.ArgumentParser(
        description="Run a HiWonder Action Group via ActionGroupRunner action server."
    )
    parser.add_argument(
        "name",
        help="Action group name (without .d6a extension), e.g. grab_forward_pro, start, transfer_to_left, wave_pro",
    )
    parser.add_argument(
        "-r",
        "--repeat",
        type=int,
        default=1,
        help="Number of times to repeat the action group (default: 1)",
    )

    # Use rospy.myargv to strip out ROS remapping arguments
    args = parser.parse_args(rospy.myargv()[1:])
    return args


def main():
    rospy.init_node("run_action_group_cli")

    args = parse_args()
    action_name = args.name + ".d6a"
    repeat = args.repeat

    rospy.loginfo(f"ActionGroupRunner client starting. name={action_name}, repeat={repeat}")

    # Create action client
    client = actionlib.SimpleActionClient(
        "ActionGroupRunner",  # matches /ActionGroupRunner/goal etc
        ActionGroupRunnerAction,
    )

    rospy.loginfo("Waiting for ActionGroupRunner action server...")
    client.wait_for_server()
    rospy.loginfo("ActionGroupRunner is available.")

    # Build goal
    goal = ActionGroupRunnerGoal()
    goal.name = action_name
    goal.repeat = repeat

    rospy.loginfo(f"Sending action group goal: name={goal.name}, repeat={goal.repeat}")
    client.send_goal(goal)

    # Wait for completion
    client.wait_for_result()
    result = client.get_result()
    state = client.get_state()
    status_text = client.get_goal_status_text()

    rospy.loginfo("Action finished.")
    rospy.loginfo(f"  state: {state} ({GoalStatus.to_string(state)})")
    rospy.loginfo(f"  status_text: {status_text}")
    rospy.loginfo(f"  result: {result}")


if __name__ == "__main__":
    main()
