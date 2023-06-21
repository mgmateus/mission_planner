import rospy
import smach
from flight_pkg.base_controller import BaseController


class Armed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['armed', 'wait_for_arming'])
        rospy.Subscriber("/mavros/state", State, self._state_callback)
        
        self.__armed = False  
        self.__base_controller = BaseController()   
    
    def _state_callback(self, msg):
        self.__armed = msg.armed

    def execute(self, status):
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

    def execute(self, status):
        if not self.__is_started:
            if self.__mode != 'STABILIZE':
                self.__base_controller.takeoff(altitude=self.__altitude)
                self.__is_started = True
                return 'wait_for_altitude'
            return 'wait_for_autonomous_mode'
        
        if status.ready:
            return 'take_off'
        return 'wait_for_altitude'
"""

class TakeOff(smach.State):
    def __init__(self, altitude):
        smach.State.__init__(self, outcomes = ['take_off', 'wait_for_autonomous_mode'])
        
        rospy.Subscriber("/mavros/state", State, self._state_callback)
        
        self.__mode = 'STABILIZE' 
        self.__altitude = altitude
        self.__base_controller = BaseController()   

    def _state_callback(self, msg):
        self.__mode = msg.mode

    def execute(self):
        if self.__mode != 'STABILIZE':
            self.__base_controller.takeoff(altitude=self.__altitude)
            return 'take_off'
        return 'wait_for_autonomous_mode'
    

class Land(smach.State):
    def __init__(self, altitude):
        smach.State.__init__(self, outcomes = ['land'])
        self.__base_controller = BaseController()   

    def execute(self):
        self.__base_controller.land()
        return 'land'

class RangeFinderCheck(smach.state):
    def __init__(self, goal_altitude):
        smach.State.__init__(self, outcomes = ['its_flying', 'wait_for_altitude'], output_keys = ['ready'])
        rospy.Subscriber(rospy.Subscriber("/mavros/rangefinder/rangefinder", Range, self._range_finder_callback))

        self.__altitude = 0.0
        self.__goal_altitude = goal_altitude
        self.__base_controller = BaseController()   

    def _range_finder_callback(self, msg):
        self.__altitude = msg.range

    def execute(self, status):
        if (self.__goal_altitude - self.__altitude) <= 0.1:
            status.ready = True
            return 'its_flying'
        else:
            status.ready = False
            return 'wait_for_altitude'
       



















