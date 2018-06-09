#! /usr/bin/env python
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot, QTimer

import rospy

from iarc7_abstract.msg import ControlButtons

class App(QWidget):
 
    def __init__(self):
        super(App, self).__init__()
        self.title = 'IARC Control Buttons'
        self.left = 0
        self.top = 0
        self.width = 200
        self.height = 200
        self.initUI()
 
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
 
        button = QPushButton('START', self)
        button.setToolTip('Start abstract execution')
        button.move(55,70) 
        button.clicked.connect(self.on_click)
 
        self.show()
 
    @pyqtSlot()
    def on_click(self):
        rospy.loginfo('START button clicked')
        msg = ControlButtons()
        msg.start = True
        start_publisher.publish(msg)

def timer_callback():
    if rospy.is_shutdown():
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('control_buttons_gui')
    start_publisher = rospy.Publisher('/control_buttons', 
                                      ControlButtons,
                                      queue_size = 10)
    rospy.set_param('/control_buttons_active', True)

    app = QApplication(sys.argv)

    timer = QTimer()
    timer.start(100) 
    timer.timeout.connect(timer_callback)

    ex = App()
    sys.exit(app.exec_())
