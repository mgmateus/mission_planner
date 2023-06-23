#!/usr/bin/env python3
"""This is an example of how to do mission planning using BaseController. Do not run this module!"""
import rospy
from base_states import *

if __name__ == "__main__":
    rospy.init_node("mission_plane_0_node")

    # Create a SMACH state machine
    sm_mission = smach.StateMachine(outcomes=['mission_finished'])

    with sm_mission:
        altitude = 1.0

        sm_its_flying = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])
        
        with sm_its_flying:
            smach.StateMachine.add("ARMED", Armed(),
                                   transitions={
                                       "wait_for_arming":"ARMED",
                                        "armed" : "WAIT_FOR_ALTITUDE"
                                    })
            
            con_wait_for_altitude = smach.Concurrence(outcomes=['wait_for_altitude','ready_to_nav'],
                                    outcome_map={'ready_to_nav':
                                        {'TAKEOFF':'take_off',
                                        'READ_ALTITUDE':'ready'}})
            
            with con_wait_for_altitude:
                smach.Concurrence.add('TAKEOFF', TakeOff(altitude),
                                      transitions={
                                        'wait_for_autonomous_mode':'TAKEOFF',
                                        'wait_for_altitude' : 'TAKEOFF'
                                    })
                
                smach.Concurrence.add('READ_ALTITUDE', RangeFinderCheck(altitude),
                                      transitions={
                                        'wait_for_altitude' : 'READ_ALTITUDE'
                                    })
            

            smach.StateMachine.add("WAIT_FOR_ALTITUDE", con_wait_for_altitude,
                                   transitions={
                                        'wait_for_altitude' : "WAIT_FOR_ALTITUDE",
                                        'ready_to_nav' : "LAND"
                                    })
        
            smach.StateMachine.add("LAND", Land(),
                                   transitions={
                                        "land" : "succeeded"
                                    })
        
    
    # Execute SMACH plan
    outcome = sm_mission.execute()

