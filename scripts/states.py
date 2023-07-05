import rospy
import smach
from typing import *
from mission_planner.base_controller import BaseController

from mavros_msgs.msg import State
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped


        
class Armed(smach.State, BaseController):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_arming', 'armed'])
        BaseController.__init__(self)

    def execute(self, userdata):
        if self.is_armed():
            return 'armed'
        return 'wait_for_arming'
    

class TakeOff(smach.State, BaseController):
    def __init__(self, target_height : float):
        smach.State.__init__(self, outcomes = ['wait_for_autonomous_mode', 'wait_for_height'])
        BaseController.__init__(self)

        self.__target_height = target_height

    def execute(self, userdata):
        if self.get_mode() != 'STABILIZE':
            self.takeoff(altitude=self.__target_height)
            return 'wait_for_height'
        
        return 'wait_for_autonomous_mode'

class Land(smach.State, BaseController):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['land'])
        BaseController.__init__(self)

    def execute(self, userdata):
        self.land()
        return 'land'
    
class Navigate(smach.State, BaseController):
    def __init__(self, position : List):
        smach.State.__init__(self, outcomes = ['wait_for_position'])
        BaseController.__init__(self)

        self.__position = position
        self.__is_runing = False
        
    def execute(self, userdata):
        rospy.logwarn(self.__position)
        x, y, z = self.__position 
        self.set_position(position_x= x, position_y= y, position_z= z)
        return 'wait_for_position'
    
class Yaw(smach.State, BaseController):
    def __init__(self, yaw= None):
        smach.State.__init__(self, outcomes = ['turned', 'wait_for_yaw'], input_keys = ['yaw', 'ready'])
        BaseController.__init__(self)

        self.__yaw = yaw
        self.__is_runing = False
        
    def execute(self, userdata):
        try:
            if userdata.ready:
                self.__is_runing = False
                userdata.ready = False
                return 'turned'
        except:
            pass
        
        if self.__is_runing:
            return 'wait_for_yaw'
        
        yaw = self.__yaw or userdata.yaw
        
        self.save_position_to_yaw()
        self.set_yaw(yaw)
        self.__is_runing = True
        return 'wait_for_yaw'
    






       



















