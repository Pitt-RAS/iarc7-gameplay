#! /usr/bin/env python
import sys
import rospy
from iarc7_msgs.msg import OdometryArray
from iarc7_msgs.msg import RoombaDetectionFrame
import actionlib
import tf2_ros
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

def track_roomba_land():
    safety_client = SafetyClient('track_roomba_abstract')
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

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=0.0, z_position=1.5)
    # Sends the goal to the action server.
    client.send_goal(goal)

    rospy.sleep(2.0)
    client.cancel_goal()
    rospy.logwarn("Dont ascending")

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=1.0, y_velocity=0.0, z_position=1.5)
    # Sends the goal to the action server.
    client.send_goal(goal)

    search_start_time = rospy.Time.now()
    rate = rospy.Rate(30)
    roomba_detected = False
    while True:
        if rospy.Time.now() - search_start_time > rospy.Duration(3.0):
            rospy.loginfo("Searching for Roomba timed out")
            break
        elif len(roomba_detections_array) > 0:
            roomba_detected = True
            rospy.loginfo("FOUND ROOMBA")
            break
        rate.sleep()

    client.cancel_goal()
    rospy.logwarn("Translation to roomba canceled")

    if roomba_detected:
        # change element in array to test diff roombas
        roomba_id = roomba_array.data[0].child_frame_id 
        roomba_id = roomba_id [0:len(roomba_id)-10]

        # Test tracking
        goal = QuadMoveGoal(movement_type="track_roomba", frame_id=roomba_id, 
            time_to_track=10.0, x_overshoot=0.0, y_overshoot=0.0)
        # Sends the goal to the action server.
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        client.wait_for_result()
        rospy.logwarn("Track Roomba success: {}".format(client.get_result()))
    else:
        rospy.logerr("Roomba not found while searching, returning")

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=-0.5, y_velocity=0.0, z_position=1.5)
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(3.0)
    client.cancel_goal()
    rospy.logwarn("Translation to roomba canceled")

    goal = QuadMoveGoal(movement_type="land")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Land success: {}".format(client.get_result()))

def _receive_roomba_status(data):
    global roomba_array
    roomba_array = data

roomba_detections_array = []
def _receive_roomba_detections(data):
    global roomba_detections_array
    roomba_detections_array = data.roombas

if __name__ == '__main__':
    roomba_array = []
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('track_roomba_abstract')
    _roomba_status_sub = rospy.Subscriber('roombas',
                     OdometryArray, _receive_roomba_status)
    _roomba_vision_sub = rospy.Subscriber('detected_roombas',
                                          RoombaDetectionFrame,
                                          _receive_roomba_detections)
    track_roomba_land()
    rospy.spin()
