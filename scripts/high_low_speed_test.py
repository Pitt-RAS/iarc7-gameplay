#! /usr/bin/env python
import sys
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

def velocity_test():
    safety_client = SafetyClient('high_low_speed_test_abstract')
    # Since this abstract is top level in high_low_speed_test control chain there is no need to check
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

    X_VELOCITY = 3.0
    Y_VELOCITY = 4.0
    X_DELAY = 1.0
    #Y_DELAY = 0.0
    STOP_DELAY = 2.0

    # Test takeoff
    goal = QuadMoveGoal(movement_type="takeoff")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Takeoff success: {}".format(client.get_result()))

    for i in range(1,3):
        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=X_VELOCITY, y_velocity=0.0, z_position=0.8)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(X_DELAY)
        client.cancel_goal()
        rospy.logwarn("Translation {} canceled".format(i*2))

        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=0.0, z_position=0.8)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(STOP_DELAY)
        client.cancel_goal()
        rospy.logwarn("Stop {} canceled".format(i*2))

        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=-X_VELOCITY, y_velocity=0.0, z_position=0.8)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(X_DELAY)
        client.cancel_goal()
        rospy.logwarn("Translation {} canceled".format(i*2+1))

        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=0.0,  z_position=0.8)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(STOP_DELAY)
        client.cancel_goal()
        rospy.logwarn("Stop {} canceled".format(i*2+1))

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=0.0,  z_position=0.6)
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(3.0)
    client.cancel_goal()
    rospy.logwarn("Stop {} canceled".format(6))

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0, y_velocity=-0.4, z_position=0.6)
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(2.0)
    client.cancel_goal()
    rospy.logwarn("Translation {} canceled".format(7))

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=0.0,  z_position=0.6)
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(STOP_DELAY)
    client.cancel_goal()
    rospy.logwarn("Stop {} canceled".format(8))

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.4, y_velocity=0.0, z_position=0.6)
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(3.0)
    client.cancel_goal()
    rospy.logwarn("Translation {} canceled".format(9))

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=0.0,  z_position=0.6)
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(STOP_DELAY)
    client.cancel_goal()
    rospy.logwarn("Stop {} canceled".format(10))

    # Test land
    goal = QuadMoveGoal(movement_type="land")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Land success: {}".format(client.get_result()))

if __name__ == '__main__':
    try:
        rospy.init_node('high_low_speed_test')
        velocity_test()
        rospy.spin()

    except Exception, e:
        rospy.logfatal("Error in motion planner while running.")
        rospy.logfatal(str(e))
        raise
    finally:
        rospy.signal_shutdown("Velocity Test abstract shutdown")
