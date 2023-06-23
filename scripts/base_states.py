import rospy
import smach
from mission_planner.base_controller import BaseController

from mavros_msgs.msg import State
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped

class Armed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_arming', 'armed'])
        rospy.Subscriber("/mavros/state", State, self._state_callback)
        
        self.__armed = False  
        self.__base_controller = BaseController()   
    
    def _state_callback(self, msg):
        self.__armed = msg.armed

    def execute(self, userdata):
        if self.__armed:
            return 'armed'
        
        return 'wait_for_arming'
"""
class TakeOff(smach.State):
    def __init__(self, altitude):
        smach.State.__init__(self, outcomes = ['take_off', 'wait_for_autonomous_mode', 'wait_for_altitude'], 
                             input_keys = ['armed', 'mode', 'ready'])
        
        rospy.Subscriber("/mavros/state", State, self._state_callback)
        
        self.__mode = 'STABILIZE' 
        self.__is_started = False
        self.__altitude = altitude
        
        self.__base_controller = BaseController()   

    def _state_callback(self, msg):
        self.__mode = msg.mode

    def execute(self, userdata):
        if not self.__is_started:
            if self.__mode != 'STABILIZE':
                self.__base_controller.takeoff(altitude=self.__altitude)
                self.__is_started = True
                return 'wait_for_altitude'
            return 'wait_for_autonomous_mode'
        
        if userdata.ready:
            return 'take_off'
        return 'wait_for_altitude'
"""

class TakeOff(smach.State):
    def __init__(self, goal_altitude):
        smach.State.__init__(self, outcomes = ['wait_for_autonomous_mode', 'wait_for_altitude', 'take_off'], input_keys = ['ready'])
        rospy.Subscriber("/mavros/state", State, self._state_callback)
        
        self.__mode = 'STABILIZE' 
        self.__goal_altitude = goal_altitude
        self.__init = False
        self.__base_controller = BaseController()   

    def _state_callback(self, msg):
        self.__mode = msg.mode

    def execute(self, userdata):
        if userdata.ready:
            return 'take_off'
        
        if self.__init:
            return 'wait_for_altitude'
        
        if self.__mode != 'STABILIZE':
            self.__base_controller.takeoff(altitude=self.__goal_altitude)
            self.__init = True
            
            return 'wait_for_altitude'
            
        return 'wait_for_autonomous_mode'

class RangeFinderCheck(smach.State):
    def __init__(self, goal_altitude):
        smach.State.__init__(self, outcomes = ['wait_for_altitude', 'ready'], output_keys = ['ready'])
        rospy.Subscriber("/mavros/rangefinder/rangefinder", Range, self._range_finder_callback)

        self.__altitude = 0.0
        self.__goal_altitude = goal_altitude
        self.__base_controller = BaseController()   

    def _range_finder_callback(self, msg):
        self.__altitude = msg.range

    def execute(self, userdata):
        rospy.logwarn(f"altitude: {self.__altitude}, falta: {self.__goal_altitude - self.__altitude}")
        if (self.__goal_altitude - self.__altitude) <= 0.1:
            userdata.ready = True
            return 'ready'
        else:
            userdata.ready = False
            return 'wait_for_altitude'

class Land(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['land'])
        self.__base_controller = BaseController()   

    def execute(self, userdata):
        self.__base_controller.land()
        return 'land'


       



















