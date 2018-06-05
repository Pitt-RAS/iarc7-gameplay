#! /usr/bin/env python
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient
from iarc7_abstract.roomba_request_executer import (RoombaRequestExecuter,
                                     RoombaRequest,
                                     RoombaRequestExecuterState)

def test_rre():
    safety_client = SafetyClient('roomba_request_executer_test_abstract')
    # Since this abstract is top level in the control chain there is no need to
    # check for a safety state. We can also get away with not checking for a
    # fatal state since all nodes below will shut down.
    assert(safety_client.form_bond())
    if rospy.is_shutdown(): return

    # Creates the SimpleActionClient, passing the type of the action
    # (QuadMoveAction) to the constructor. (Look in the action folder)
    client = actionlib.SimpleActionClient("motion_planner_server",
                                          QuadMoveAction)

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

    while roomba_array is None and not rospy.is_shutdown():
        rospy.sleep(2)

    # change element in array to test diff roombas
    roomba_id = roomba_array.data[-1].child_frame_id

    RoombaRequestExecuter.init('motion_planner_server')

    roomba_request = RoombaRequest(roomba_id, RoombaRequest.BUMP, 5)

    RoombaRequestExecuter.run(roomba_request, _receive_roomba_executer_status)

    while RoombaRequestExecuter.has_running_task():
        rospy.logwarn('Current Roomba Request Executer state: ' + executer_state.name)
        rospy.sleep(2)

    if executer_state == RoombaRequestExecuterState.SUCCESS:
        rospy.logwarn("Roomba request executer completed successfully")
    else:
        rospy.logerr("Roomba request executer failed")

def _receive_roomba_status(data):
    global roomba_array
    roomba_array = data

def _receive_roomba_executer_status(data):
    global executer_state
    executer_state = data
    rospy.logerr_throttle(60, 'Updated state: ' + executer_state.name)

if __name__ == '__main__':
    executer_state = None
    roomba_array = None
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('roomba_request_executer_test_abstract')
    _roomba_status_sub = rospy.Subscriber('roombas',
                     OdometryArray, _receive_roomba_status)

    test_rre()
    rospy.spin()
