import rospy
import smach
from flight_pkg.base_controller import BaseController


class Armed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['armed', 'wait_for_auto_mode'], output_keys = ['armed', 'mode'])
        rospy.Subscriber("/mavros/state", State, self._state_callback)
        
        self.__armed = False  
        self.__mode = 'STABILIZE' 
        self.__base_controller = BaseController()   
    
    def _state_callback(self, msg):
        self.__armed = msg.armed
        self.__mode = msg.mode

    def execute(self, status):
        if self.__armed:
            status.armed = self.__armed
            status.mode = self.__mode
            return 'armed'
        return 'wait_for_auto_mode' 


class TakeOff(smach.State):
    def __init__(self, altitude):
        smach.State.__init__(self, outcomes = ['takeoff', 'wait_for_auto_mode', 'wait_for_altitude'], input_keys = ['armed', 'mode', 'ready_altitude'])
        
        self.__altitude = altitude
        self.__base_controller = BaseController()   

    def execute(self, status):
        if status.armed and status.mode != 'STABILIZE':
            self.__base_controller.takeoff(altitude=self.__altitude)
            if 
            return 'takeoff'
        return  'wait_for_auto_mode'

class RangeFinderCheck(smach.state):
    def __init__(self, goal_altitude):
        smach.State.__init__(self, outcomes = ['ready_altitude', 'wait_for_altitude'], output_keys = ['ready_altitude'])
        rospy.Subscriber(rospy.Subscriber("/mavros/rangefinder/rangefinder", Range, self._range_finder_callback)

        self.__altitude = 0.0
        self.__goal_altitude = goal_altitude
        self.__base_controller = BaseController()   

    def _range_finder_callback(self, msg):
        self.__altitude = msg.range

    def execute(self, status):
        if (self.__goal_altitude - self.__altitude) <= 0.1:
            status.ready_altitude = True
            return 'ready_altitude'
        else:
            status.ready_altitude = False
            return 'wait_for_altitude'
       



















