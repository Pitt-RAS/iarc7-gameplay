#! /usr/bin/env python
import sys
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

def hit_roomba_land():
    safety_client = SafetyClient('block_roomba_abstract')
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
    rospy.logwarn("Takeoff success: {}".format(client.get_result()))

    rospy.sleep(2.0)

    # change element in array to test diff roombas
    roomba_id = roomba_array.data[5].child_frame_id 

    # test going to the roomba
    goal = QuadMoveGoal(movement_type="go_to_roomba", frame_id=roomba_id)
    # Send the goal to the action server
    client.send_goal(goal)
    # Waits for the server to finihs performing the action
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn('Go To Roomba success: {}'.format(client.get_result()))

    # Test tracking
    goal = QuadMoveGoal(movement_type="track_roomba", frame_id=roomba_id,
        time_to_track=5.0, x_overshoot=.5, y_overshoot=.5)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Track Roomba success: {}".format(client.get_result()))

     # Test tracking
    goal = QuadMoveGoal(movement_type="block_roomba", frame_id = roomba_id)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Block Roomba success: {}".format(client.get_result()))

    goal = QuadMoveGoal(movement_type="height_recovery")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Height Recovery success: {}".format(client.get_result()))

def _receive_roomba_status(data):
    global roomba_array
    roomba_array = data

if __name__ == '__main__':
    roomba_array = []
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('hit_roomba_abstract')

    _roomba_status_sub = rospy.Subscriber('roombas',
                     OdometryArray, _receive_roomba_status)

    hit_roomba_land()
    rospy.spin()
