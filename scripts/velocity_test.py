#! /usr/bin/env python
import sys
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

def velocity_test():
    safety_client = SafetyClient('velocity_test_abstract')
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
    X_DELAY = 5.0
    Y_DELAY = 5.0
    STOP_DELAY = 2.0
    HEIGHT = 1.5

    # Test takeoff
    goal = QuadMoveGoal(movement_type="takeoff")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Takeoff success: {}".format(client.get_result()))

    for i in range(0, 1):
        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=X_VELOCITY, y_velocity=0.0, z_position=HEIGHT)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(X_DELAY)
        client.cancel_goal()
        rospy.logwarn("Translation 1 canceled")

        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=0.0, z_position=HEIGHT)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(STOP_DELAY)
        client.cancel_goal()
        rospy.logwarn("Stop 1 canceled")

        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=-Y_VELOCITY, z_position=HEIGHT)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(Y_DELAY)
        client.cancel_goal()
        rospy.logwarn("Translation 2 canceled")

        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=0.0, z_position=HEIGHT)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(STOP_DELAY)
        client.cancel_goal()
        rospy.logwarn("Stop 2 canceled")

        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=-X_VELOCITY, y_velocity=0.0, z_position=HEIGHT)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(X_DELAY)
        client.cancel_goal()
        rospy.logwarn("Translation 3 canceled")

        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=0.0, z_position=HEIGHT)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(STOP_DELAY)
        client.cancel_goal()
        rospy.logwarn("Stop 3 canceled")

        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=Y_VELOCITY,  z_position=HEIGHT)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(Y_DELAY)
        client.cancel_goal()
        rospy.logwarn("Translation 4 canceled")

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=0.0,  z_position=HEIGHT)
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(STOP_DELAY)
    client.cancel_goal()
    rospy.logwarn("Stop 4 canceled")

    # Test land
    goal = QuadMoveGoal(movement_type="land")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Land success: {}".format(client.get_result()))

if __name__ == '__main__':
    rospy.init_node('velocity_test_abstract')
    velocity_test()
    rospy.spin()
