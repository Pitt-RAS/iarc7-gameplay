#! /usr/bin/env python
import sys
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

def test():
    safety_client = SafetyClient('invalid_task_transition_test')
    # Since this abstract is top level in the control chain there is no need to check
    # for a safety state. We can also get away with not checking for a fatal state since
    # all nodes below will shut down.
    assert safety_client.form_bond()

    # Creates the SimpleActionClient, passing the type of the action
    # (QuadMoveAction) to the constructor. (Look in the action folder)
    client = actionlib.SimpleActionClient("motion_planner_server", QuadMoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    rospy.sleep(2.0)

    # arbitrarily picked fifth roomba to test
    # also removes the "/frame_id" at end of roomba_id
    roomba_id = roomba_array.data[5].child_frame_id.split('/')[0]

    rospy.logwarn("Testing illegal state #1")

    goal = QuadMoveGoal(movement_type="track_roomba", frame_id=roomba_id,
        time_to_track=5.0, x_overshoot=0, y_overshoot=0)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    assert client.get_result().success == False

    rospy.sleep(2.0)

    # Testing below min manuever height
    rospy.logwarn("Testing illegal state #2")

    goal = QuadMoveGoal(movement_type="takeoff")
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(1.0)
    client.cancel_goal()

    rospy.logwarn("Sending goal while below minimum manuever height")

    goal = QuadMoveGoal(movement_type="hit_roomba", frame_id = roomba_id)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    assert client.get_result().success == False

    rospy.sleep(2.0)

    rospy.logwarn("RESET STATE")

    goal = QuadMoveGoal(movement_type="land")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Land success: {}".format(client.get_result()))

    rospy.sleep(2.0)

    rospy.logwarn("Testing illegal state #3")

    goal = QuadMoveGoal(movement_type="hit_roomba", frame_id = roomba_id)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    assert client.get_result().success == False

    rospy.sleep(2.0)

    rospy.logwarn("RESET STATE")

    goal = QuadMoveGoal(movement_type="takeoff")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Takeoff success: {}".format(client.get_result()))

    rospy.sleep(2.0)

    rospy.logwarn("Preparing to test illegal state #4")

    goal = QuadMoveGoal(movement_type="hit_roomba", frame_id = roomba_id)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Hit Roomba success: {}".format(client.get_result()))

    rospy.sleep(2.0)

    rospy.logwarn("Testing illegal state #4")

    goal = QuadMoveGoal(movement_type="track_roomba", frame_id=roomba_id, 
        time_to_track=5.0, x_overshoot=.5, y_overshoot=.5)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    assert client.get_result().success == False
    
    rospy.sleep(2.0)
    
    rospy.logwarn("RESET STATE")

    goal = QuadMoveGoal(movement_type="height_recovery")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Height Recovery success: {}".format(client.get_result()))

    rospy.sleep(2.0)

    rospy.logwarn("Sending Test Task")

    goal = QuadMoveGoal(movement_type="test_task", takeoff_height=0.75)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Test Task success: {}".format(client.get_result()))

    rospy.sleep(2.0)

    rospy.logwarn("RESET STATE")
    goal = QuadMoveGoal(movement_type="land")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Land success: {}".format(client.get_result()))

    rospy.logwarn("Testing is complete")
    
def _receive_roomba_status(data):
    global roomba_array
    roomba_array = data

if __name__ == '__main__':
    try:
        roomba_array = []
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        _roomba_status_sub = rospy.Subscriber('roombas', 
                         OdometryArray, _receive_roomba_status)

        rospy.init_node('invalid_task_transition_test')
        test()
        rospy.spin()
    except Exception, e:
        rospy.logfatal("Error in motion planner while running.")
        rospy.logfatal(str(e))
        raise
    finally:
        rospy.signal_shutdown("Invalid Task transition test shutdown")
