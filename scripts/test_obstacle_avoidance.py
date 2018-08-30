#! /usr/bin/env python
import sys
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

def velocity_test():
    safety_client = SafetyClient('test_obstacle_avoidance_abstract')
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

    X_VELOCITY = 0.5
    Y_VELOCITY = 0.5
    TRANSLATE_DELAY = 7.0

    # Test takeoff
    goal = QuadMoveGoal(movement_type="takeoff")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Takeoff success: {}".format(client.get_result()))

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=X_VELOCITY, y_velocity=Y_VELOCITY, z_position=0.8)
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(TRANSLATE_DELAY )
    client.cancel_goal()
    rospy.logwarn("Translation 1 canceled")

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=-X_VELOCITY, y_velocity=-Y_VELOCITY, z_position=0.8)
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(TRANSLATE_DELAY )
    client.cancel_goal()
    rospy.logwarn("Returned Home")

    # Test land
    goal = QuadMoveGoal(movement_type="land")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Land success: {}".format(client.get_result()))

if __name__ == '__main__':
    rospy.init_node('test_obstacle_avoidance_abstract')
    velocity_test()
    rospy.spin()
