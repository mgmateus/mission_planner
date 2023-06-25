#!/usr/bin/env python3
"""This is an example of how to do mission planning using BaseController. Do not run this module!"""
import rospy
from states import *

if __name__ == "__main__":
    rospy.init_node("mission_plane_0_node")

    # Create a SMACH state machine
    sm_mission = smach.StateMachine(outcomes=['mission_finished'])

    with sm_mission:
        altitude = 1.0
        position = [1.0, 0.0, 1.0]

        sm_its_flying = smach.StateMachine(outcomes=["succeeded"])
        
        with sm_its_flying:

            con_wait_for_height = smach.Concurrence(outcomes=['wait_for_height','ready_to_nav'],
                                    default_outcome = 'wait_for_height',
                                    outcome_map={
                                        'ready_to_nav': {'TAKEOFF':'take_off','READ_HEIGHT':'ready'}
                                    })
            
            with con_wait_for_height:
                smach.Concurrence.add('READ_HEIGHT', RangeFinderCheck(altitude))
                smach.Concurrence.add('TAKEOFF', TakeOff(altitude))

            con_wait_for_position = smach.Concurrence(outcomes=['wait_for_position','ready_to_land'],
                                    default_outcome = 'wait_for_position',
                                    outcome_map={
                                        'ready_to_land': {'NAVIGATION':'sailed','READ_POSITION':'ready'}
                                    })
            
            with con_wait_for_position:
                smach.Concurrence.add('READ_POSITION', PositionCheck(target_position=position))
                smach.Concurrence.add('NAVIGATION', Navigate(position))


            
            #######################################
            smach.StateMachine.add("ARMED", Armed(),
                                   transitions={
                                       "wait_for_arming":"ARMED",
                                        "armed" : "WAIT_FOR_HEIGHT"
                                    })
            
            
            smach.StateMachine.add("WAIT_FOR_HEIGHT", con_wait_for_height,
                                   transitions={
                                        'wait_for_height' : "WAIT_FOR_HEIGHT",
                                        'ready_to_nav' : "WAIT_FOR_POSITION"
                                    })
            
            smach.StateMachine.add("WAIT_FOR_POSITION", con_wait_for_position,
                                   transitions={
                                        'wait_for_position' : "WAIT_FOR_POSITION",
                                        'ready_to_land' : "LAND"
                                    })
        
            smach.StateMachine.add("LAND", Land(),
                                   transitions={
                                        "land" : "succeeded"
                                    })
            #######################################

        smach.StateMachine.add("ITS_FLING", sm_its_flying,
                                   transitions={
                                        "succeeded" : "mission_finished"
                                    })
        
    
    # Execute SMACH plan
    outcome = sm_mission.execute()

