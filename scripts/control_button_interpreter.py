#! /usr/bin/env python
import rospy
from iarc7_abstract.msg import ControlButtons

class ControlButtonInterpreter(object):
    def __init__(self):
        self._control_buttons_active = rospy.get_param('/control_buttons_active', False)
        if self._control_buttons_active:
            self._control_buttons = ControlButtons()
            self._control_buttons_sub = rospy.Subscriber('/control_buttons', 
                                                 ControlButtons,
                                                 self._receive_control_buttons)

    def _receive_control_buttons(self, msg):
        self._control_buttons = msg

    def get_start(self):
        if self._control_buttons_active:
            return self._control_buttons.start
        else:
            return True

    def wait_for_start(self):
        if self._control_buttons_active:
            rate = rospy.Rate(10)
            while not self._control_buttons.start \
                  and not rospy.is_shutdown():
                rospy.loginfo_throttle(0.5, 'Waiting for start signal')
                rate.sleep()
            rospy.loginfo('Start signal received')
        else:
            rospy.loginfo('Start signal disabled, starting immediately')
