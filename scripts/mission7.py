#! /usr/bin/env python
import sys
import rospy
import math
import numpy as np

import actionlib
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient
from iarc7_msgs.msg import OdometryArray
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus

from iarc7_abstract.arena_position_estimator import ArenaPositionEstimator

TRANSLATION_HEIGHT = 1.5


SEARCH_POINTS = np.asarray(
(
    ( 0.0,  0.0),   # Center
    ( 0.0,  7.5),   # Mid left
    (-7.5,  7.5),   # Bottom left
    (-7.5, -7.5),   # Bottom right
    ( 0.0, -7.5),   # Mid right
    ( 7.5,  0.0),   # Top middle
))

def target_roomba_law(roombas):
    return roombas[0]

def construct_xyz_goal(arena_pos, height=TRANSLATION_HEIGHT):
    map_pos = arena_position_estimator.arena_to_map(arena_pos)
    return QuadMoveGoal(movement_type="xyztranslate", x_position=map_pos[0], y_position=map_pos[1], z_position=height)

def roomba_distance(roomba_odom, drone_odom):
    x_diff = drone_odom.pose.pose.position.x - roomba_odom.pose.pose.position.x
    y_diff = drone_odom.pose.pose.position.y - roomba_odom.pose.pose.position.y
    return math.sqrt(x_diff**2 + y_diff**2)

class Mission7(object):
    def __init__(self):
        self.safety_client = SafetyClient('mission7')
        # Since this abstract is top level in the control chain there is no need to check
        # for a safety state. We can also get away with not checking for a fatal state since
        # all nodes below will shut down.
        assert(self.safety_client.form_bond())
        if rospy.is_shutdown(): return

        # Creates the SimpleActionClient, passing the type of the action
        # (QuadMoveAction) to the constructor. (Look in the action folder)
        self._client = actionlib.SimpleActionClient("motion_planner_server", QuadMoveAction)

        # Waits until the action server has started up and started
        # listening for goals.
        self._client.wait_for_server()
        if rospy.is_shutdown(): return

        self._avail_roomba = None
        self._search_state = 0
        self._roomba_sub = rospy.Subscriber('/roombas', OdometryArray, self.roomba_callback)
        self._odom_sub = rospy.Subscriber('/odometry/filtered/', Odometry, self.odom_callback)

    def roomba_callback(self, data):
        # Sort roombas by their distance to the drone
        self._avail_roombas = sorted([(roomba_distance(r, self._odom), r) for r in data.data])

    def odom_callback(self, data):
        self._odom = data

    def arena_translate(self, arena_pos, height=TRANSLATION_HEIGHT):
        map_pos = construct_xyz_goal(arena_pos, height=height)
        print 'MAP POS', map_pos
        self._client.send_goal(map_pos)
        self._client.wait_for_result()
        rospy.loginfo('XYZ Translate success: {}'.format(self._client.get_result()))

    def begin_translate(self, arena_pos, height=TRANSLATION_HEIGHT):
        map_pos = construct_xyz_goal(arena_pos, height=height)
        self._client.send_goal(map_pos)

    def basic_goal(self, goal):
        self._client.send_goal(QuadMoveGoal(movement_type=goal))
        self._client.wait_for_result()
        rospy.loginfo('{} success: {}'.format(goal, self._client.get_result()))

    def wait_for_roomba(self):
        rate = rospy.Rate(30)
        while True:
            state = self._client.get_state()
            if (state == GoalStatus.ABORTED or state == GoalStatus.SUCCEEDED):
                return None
            if self._avail_roombas is not None and len(self._avail_roombas) > 0:
                target_roomba = target_roomba_law(self._avail_roombas)
                if target_roomba is not None:
                    return target_roomba
            rate.sleep()

    def search_for_roomba(self):
        # Decide which waypoint to go to
        if self._search_state != 0:
            self._search_state = 1

        # Execute search waypoints
        while True:
            self.begin_translate(SEARCH_POINTS[self._search_state])
            roomba = self.wait_for_roomba()
            if roomba is None:
                self._search_state = self._search_state + 1 if self._search_state + 1 < SEARCH_POINTS.shape[0] else 1
            else:
                break

        self._search_state = 1
        return roomba

    def attempt_mission7(self):
        # Takeoff
        self.basic_goal('takeoff')

        roomba = self.search_for_roomba()

        rospy.logerr('GOT A ROOMBA')
        rospy.logerr(roomba)

        self.basic_goal('land')

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('mission7')
    arena_position_estimator = ArenaPositionEstimator()

    mission7 = Mission7()
    mission7.attempt_mission7()

    rospy.spin()
