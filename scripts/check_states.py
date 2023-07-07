import rospy
import smach
import time
from mission_planner.base_controller import BaseController

from mavros_msgs.msg import State
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped


class RangeFinderCheck(smach.State, BaseController):
    def __init__(self, target_height : float, threshold : float = 0.1):
        smach.State.__init__(self, outcomes = ['wait_for_height', 'ready'], output_keys = ['waypoint'])
        BaseController.__init__(self)

        self.__target_height = target_height  
        self.__threshold = threshold

    def execute(self, userdata):
        if self.is_target_height(self.__target_height, self.__threshold):
            userdata.waypoint = self.get_current_position()
            return 'ready'
        else:
            return 'wait_for_height'
        
class PositionCheck(smach.State, BaseController):
    def __init__(self, target_position : float, threshold : float = 0.02):
        smach.State.__init__(self, outcomes = ['wait_for_position', 'ready'])
        BaseController.__init__(self)

        self.__target_position = target_position 
        self.__threshold = threshold

    def execute(self, userdata):
        position = self.__target_position 
        if self.is_target_position(position, self.__threshold):
            return 'ready'
        else:
            return 'wait_for_position'
        
class WaypointCheck(smach.State, BaseController):
    def __init__(self, threshold : float = 0.1):
        smach.State.__init__(self, outcomes = ['wait_for_waypoint', 'ready'], input_keys = ['waypoint'])
        BaseController.__init__(self)

        self.__threshold = threshold

    def execute(self, userdata):
        if self.is_target_position(userdata.waypoint, self.__threshold):
            return 'ready'
        else:
            return 'wait_for_waypoint'
        
class TurnaroundCheck(smach.State, BaseController):
    def __init__(self, threshold= 5.0):
        smach.State.__init__(self, outcomes = ['wait_for_turne', 'ready'], input_keys = ['turne'])
        BaseController.__init__(self)

        self.__threshold = threshold

    def execute(self, userdata):
        
        if self.is_target_turne(userdata.turne, self.__threshold):
            return 'ready'
        else:
            return 'wait_for_turne'