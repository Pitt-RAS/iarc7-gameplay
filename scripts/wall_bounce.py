#! /usr/bin/env python
import sys
import rospy
import math
import random

from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

from iarc7_abstract.arena_position_estimator import ArenaPositionEstimator

def wall_bounce():
    safety_client = SafetyClient('wall_bounce_abstract')
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

    VELOCITY = 0.5
    STOP_DELAY = 2.0
    # Start going forward
    angle = 0
    distance_to_wall_threshold = 2.0
    last_wall_hit = -1

    # Test takeoff
    goal = QuadMoveGoal(movement_type="takeoff")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Takeoff success: {}".format(client.get_result()))
    rospy.sleep(0.5)

    for i in range(0, 100):

        # Go in a direction until the distance to any wall (that was not the last wall hit) is too small
        x_vel = math.cos(angle) * VELOCITY
        y_vel = math.sin(angle) * VELOCITY
        rospy.logwarn('SET A VELOCITY')
        rospy.logwarn('Changing direction, new angle: {} degrees'.format(angle * 180.0 / math.pi))
        goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=x_vel, y_velocity=y_vel, z_position=1.5)
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.sleep(0.1)

        CONE_WIDTH_RAD = 30 * math.pi / 180.0
        CONE_OFFSET_RAD = ((math.pi/2) - CONE_WIDTH_RAD) / 2.0

        rate = rospy.Rate(30)
        while True:
            if (last_wall_hit != 0 and last_wall_hit != 3) and arena_position_estimator.distance_to_left() < distance_to_wall_threshold:
                last_wall_hit = 0
                rospy.logerr('LEFT WALL HIT')
                # random angle pointed SE
                angle = -((random.random() * CONE_WIDTH_RAD) + CONE_OFFSET_RAD) - (math.pi/2)
                break

            if (last_wall_hit != 1 and last_wall_hit != 2) and arena_position_estimator.distance_to_right() < distance_to_wall_threshold:
                last_wall_hit = 1
                rospy.logerr('RIGHT WALL HIT')
                # random angle pointed NW
                angle = ((random.random() * CONE_WIDTH_RAD) + CONE_OFFSET_RAD)
                break

            if (last_wall_hit != 2 and last_wall_hit != 0) and arena_position_estimator.distance_to_top() < distance_to_wall_threshold:
                last_wall_hit = 2
                rospy.logerr('TOP WALL HIT')
                # random angle pointed SW
                angle = ((random.random() * CONE_WIDTH_RAD) + CONE_OFFSET_RAD) + (math.pi/2)
                break

            if (last_wall_hit != 3 and last_wall_hit != 1) and arena_position_estimator.distance_to_bottom() < distance_to_wall_threshold:
                last_wall_hit = 3
                rospy.logerr('BOTTOM WALL HIT')
                # random angle pointed NE
                angle = -((random.random() * CONE_WIDTH_RAD) + CONE_OFFSET_RAD)
                break

            rate.sleep()

        client.cancel_goal()

    goal = QuadMoveGoal(movement_type="velocity_test", x_velocity=0.0, y_velocity=0.0,  z_position=1.5)
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.sleep(STOP_DELAY)
    client.cancel_goal()
    rospy.logwarn("Stopped")

    # Test land
    goal = QuadMoveGoal(movement_type="land")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if rospy.is_shutdown(): return
    rospy.logwarn("Land success: {}".format(client.get_result()))

if __name__ == '__main__':
    rospy.init_node('wall_bounce_abstract')
    arena_position_estimator = ArenaPositionEstimator()
    wall_bounce()
    rospy.spin()
