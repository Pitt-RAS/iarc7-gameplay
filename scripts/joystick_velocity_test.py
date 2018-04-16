#! /usr/bin/env python
import sys
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

def velocity_test():
    safety_client = SafetyClient('joystick_velocity_test_abstract')
    # Since this abstract is top level in the control chain there is no need to check
    # for a safety state. We can also get away with not checking for a fatal state since
    # all nodes below will shut down.
    assert(safety_client.form_bond())

    # Creates the SimpleActionClient, passing the type of the action
    # (QuadMoveAction) to the constructor. (Look in the action folder)
    client = actionlib.SimpleActionClient("motion_planner_server", QuadMoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    rospy.sleep(2.0)

    # Test takeoff
    goal = QuadMoveGoal(movement_type="takeoff")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Takeoff success: {}".format(client.get_result()))

    goal = QuadMoveGoal(movement_type="joystick_velocity_task")
    # Sends the goal to the action server.
    client.send_goal(goal)
    client.wait_for_result()
    rospy.logwarn("joystick_velocity_task finished")

    goal = QuadMoveGoal(movement_type="land")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Land success: {}".format(client.get_result()))

if __name__ == '__main__':
    try:
        rospy.init_node('joystick_velocity_test_abstract')
        velocity_test()
        rospy.spin()

    except Exception, e:
        rospy.logfatal("Error in joystick_velocity_test_abstract while running.")
        rospy.logfatal(str(e))
        raise
    finally:
        rospy.signal_shutdown("joystick_velocity_test_abstract shutdown")
