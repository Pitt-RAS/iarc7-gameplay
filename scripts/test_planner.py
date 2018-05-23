#! /usr/bin/env python
import sys
import rospy
import actionlib
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

def test_planner():
    safety_client = SafetyClient('test_planner_abstract')
    # Since this abstract is top level in the control chain there is no need to check
    # for a safety state. We can also get away with not checking for a fatal state since
    # all nodes below will shut down.
    assert(safety_client.form_bond())
    if rospy.is_shutdown(): return

    # Creates the SimpleActionClient, passing the type of the action
    # (QuadMoveAction) to the constructor. (Look in the action folder)
    client = actionlib.SimpleActionClient("motion_planner_server", QuadMoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    if rospy.is_shutdown(): return

    rospy.sleep(2.0)

    # Test takeoff
    goal = QuadMoveGoal(movement_type="takeoff")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Takeoff success: {}".format(client.get_result().success))

    rospy.sleep(2.0)

    # planner test
    goal = QuadMoveGoal(movement_type="test_planner")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Planner success: {}".format(client.get_result().success))

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('test_planner_abstract')
    test_planner()
    rospy.spin()
