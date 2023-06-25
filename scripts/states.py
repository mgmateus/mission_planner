import rospy
import smach
from mission_planner.base_controller import BaseController

from mavros_msgs.msg import State
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped

class RangeFinderCheck(smach.State, BaseController):
    def __init__(self, target_height= None, threshold= None):
        smach.State.__init__(self, outcomes = ['wait_for_altitude', 'ready'], input_keys = ['height'], output_keys = ['ready'])
        self.__target_height = target_height  
        self.__threshold = threshold

    def execute(self, userdata):
        height = self.__target_height or userdata.height
        if self.is_target_height(height, self.__threshold):
            userdata.ready = True
            return 'ready'
        else:
            userdata.ready = False
            return 'wait_for_altitude'
        
class PositionCheck(smach.State, BaseController):
    def __init__(self, target_position= None, threshold= 0.1):
        smach.State.__init__(self, outcomes = ['wait_for_position', 'ready'], input_keys = ['position'], output_keys = ['ready'])
        self.__target_position = target_position 
        self.__threshold = threshold

    def execute(self, userdata):
        position = self.__target_position or userdata.position
        if self.is_target_position(position, self.__threshold):
            userdata.ready = True
            return 'ready'
        else:
            userdata.ready = False
            return 'wait_for_altitude'
        
class YawCheck(smach.State, BaseController):
    def __init__(self, target_yaw= None, threshold= 0.1):
        smach.State.__init__(self, outcomes = ['wait_for_yaw', 'ready'], input_keys = ['yaw'], output_keys = ['ready'])
        self.__target_yaw = target_yaw 
        self.__threshold = threshold

    def execute(self, userdata):
        yaw = self.__target_yaw or userdata.yaw
        if self.is_target_yaw(yaw, self.__threshold):
            userdata.ready = True
            return 'ready'
        else:
            userdata.ready = False
            return 'wait_for_altitude'
        
class Armed(smach.State, BaseController):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_arming', 'armed'])

    def execute(self, userdata):
        if self.is_armed():
            return 'armed'
        return 'wait_for_arming'
    

class TakeOff(smach.State, BaseController):
    def __init__(self, target_height):
        smach.State.__init__(self, outcomes = ['wait_for_autonomous_mode', 'wait_for_altitude', 'take_off'], input_keys = ['ready'])
        self.__target_height = target_height
        self.__is_runing = False 

    def execute(self, userdata):
        if userdata.ready:
            return 'take_off'
        
        if self.__is_runing:
            return 'wait_for_altitude'
        
        if self.get_mode() != 'STABILIZE':
            self.takeoff(altitude=self.__target_height)
            self.__is_runing = True
            return 'wait_for_altitude'
            
        return 'wait_for_autonomous_mode'

class Land(smach.State, BaseController):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['land'])

    def execute(self, userdata):
        self.land()
        return 'land'
    
class Navigate(smach.State, BaseController):
    def __init__(self, position= None):
        smach.State.__init__(self, outcomes = ['sailed', 'wait_for_position'], input_keys = ['position', 'ready'])

        self.__position = position
        self.__is_runing = False
        
    def execute(self, userdata):
        if userdata.ready:
            self.__is_runing = False
            return 'sailed'
        
        if self.__is_runing:
            return 'wait_for_position'
        
        x, y, z = self.__position or userdata.position
        
        self.set_position(position_x= x, position_y= y, position_z= z)
        self.__is_runing = True
        return 'wait_for_position'
    
class Spin(smach.State, BaseController):
    def __init__(self, yaw= None):
        smach.State.__init__(self, outcomes = ['turned', 'wait_for_yaw'], input_keys = ['yaw', 'ready'])

        self.__yaw = yaw
        self.__is_runing = False
        
    def execute(self, userdata):
        if userdata.ready:
            self.__is_runing = False
            return 'sailed'
        
        if self.__is_runing:
            return 'wait_for_yaw'
        
        yaw = self.__yaw or userdata.yaw
        
        self.set_yaw(yaw)
        self.__is_runing = True
        return 'wait_for_yaw'
    






       


















