import rospy
import smach
from typing import *
from mission_planner.base_controller import BaseController


        
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
        
    def execute(self, userdata):
        x, y, z = self.__position 
        self.set_position(position_x= x, position_y= y, position_z= z)
        return 'wait_for_position'

class VelocityNavigate(smach.State, BaseController): #Nao testado
    def __init__(self, x : float, z : float):
        smach.State.__init__(self, outcomes = ['wait_for_position'])
        BaseController.__init__(self)

        self.__vel = [x, z]
        
    def execute(self, userdata):
        x, z = self.__vel
        self.set_velocity(x, z)
        return 'wait_for_position'
    
class WaypointNavigate(smach.State, BaseController):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_waypoint'], input_keys = ['waypoint'])
        BaseController.__init__(self)

    def execute(self, userdata):
        rospy.logwarn(userdata.waypoint)
        x, y, z = userdata.waypoint 
        self.set_position(position_x= x, position_y= y, position_z= z)
        return 'wait_for_waypoint'
    
class StepNavigate(smach.State, BaseController):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_waypoint'], input_keys = ['height', 'step'], output_keys = ['waypoint'])
        BaseController.__init__(self)

    def execute(self, userdata):
        userdata.waypoint = self.point_from_distance_rotation(userdata.height, userdata.step)
        return 'wait_for_waypoint'
    
class Turnaround(smach.State, BaseController):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_turne'], input_keys = ['turne'])
        BaseController.__init__(self)
        
    def execute(self, userdata):
        self.save_position_to_turne()
        self.set_turne(userdata.turne)
        return 'wait_for_turne'
    






       



















