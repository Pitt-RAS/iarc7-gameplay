#! /usr/bin/env python
import sys
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

def hold_current_position_land():
    safety_client = SafetyClient('hold_position_abstract')
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

    _x_position = _drone_odometry.pose.pose.position.x - .25
    _y_position = _drone_odometry.pose.pose.position.y - .25
    _z_position = _drone_odometry.pose.pose.position.z + .1

    # Test tracking
    goal = QuadMoveGoal(movement_type="hold_position", hold_current_position = False, 
                        x_position=_x_position, y_position=_y_position, z_position=_z_position)
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(5)
    client.cancel_goal()
    rospy.logwarn("Hold Position Task canceled")

def _current_velocity_callback(data):
    global _drone_odometry 
    _drone_odometry = data

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('hold_position_abstract')

    _drone_odometry = None

    _current_velocity_sub = rospy.Subscriber(
        '/odometry/filtered', Odometry,
        _current_velocity_callback)

    hold_current_position_land()
    rospy.spin()
