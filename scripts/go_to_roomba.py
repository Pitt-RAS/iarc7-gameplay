#! /usr/bin/env python
import sys
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

def go_to_roomba_land():
    safety_client = SafetyClient('go_to_roomba_abstract')
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

    rospy.sleep(2.0)


    rospy.logwarn("Testing GoToRoomba 3 times ...")
    for i in range(3):
        # change element in array to test diff roombas
        roomba_id = roomba_array.data[i].child_frame_id
        roomba_id = roomba_id [0:len(roomba_id)-10]
        rospy.logwarn("Go To Roomba id = {0}".format(roomba_id))

        # Test going
        goal = QuadMoveGoal(movement_type="go_to_roomba", frame_id = roomba_id, ending_radius_to_roomba=0.5)
        # Sends the goal to the action server.
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        client.wait_for_result()
        rospy.logwarn("Go To Roomba {0} time(s) success: {1}".format(i+1, client.get_result()))
    rospy.logwarn("GoToRoomba 3 times all success")

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

        rospy.init_node('go_to_roomba_abstract')
        go_to_roomba_land()
        while not rospy.is_shutdown():
            pass

    except Exception, e:
        rospy.logfatal("Error in motion planner while running.")
        rospy.logfatal(str(e))
        raise
    finally:
        rospy.signal_shutdown("Takeoff and GoTo Roomba abstract shutdown")

