import rospy
import smach
from typing import *
from mission_planner.base_vision import BaseVision

class QrDetection(smach.State, BaseVision):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_qr', 'detected'])
        BaseVision.__init__(self)

    def execute(self, userdata):
        if self.is_qr_detected():
            return 'detected'
        return 'wait_for_qr'