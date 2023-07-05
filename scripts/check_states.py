import rospy
import smach
from mission_planner.base_controller import BaseController

from mavros_msgs.msg import State
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped


class RangeFinderCheck(smach.State, BaseController):
    def __init__(self, target_height : float, threshold : float = 0.15):
        smach.State.__init__(self, outcomes = ['wait_for_height', 'ready'])
        BaseController.__init__(self)

        self.__target_height = target_height  
        self.__threshold = threshold

    def execute(self, userdata):
        if self.is_target_height(self.__target_height, self.__threshold):
            return 'ready'
        else:
            userdata.ready = False
            return 'wait_for_height'
        
class PositionCheck(smach.State, BaseController):
    def __init__(self, target_position : float, threshold : float = 0.15):
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
        
class YawCheck(smach.State, BaseController):
    def __init__(self, target_yaw= None, threshold= 5.0):
        smach.State.__init__(self, outcomes = ['wait_for_yaw'], input_keys = ['yaw'], output_keys = ['position','ready'])
        BaseController.__init__(self)

        self.__target_yaw = target_yaw 
        self.__threshold = threshold

    def execute(self, userdata):
        yaw = self.__target_yaw or userdata.yaw
        
        if self.is_target_yaw(yaw, self.__threshold):
            userdata.ready = True
            rospy.logwarn(f"current: {self.get_current_yaw()}")
            return 'ready'
        else:
            userdata.ready = False
            rospy.logwarn(f"target: {yaw} ---- current: {self.get_current_yaw()}")
            return 'wait_for_yaw'