#!/usr/bin/env python3
"""This is an example of how to do mission planning using BaseController. Do not run this module!"""
import rospy
from states import *

if __name__ == "__main__":
    rospy.init_node("mission_plane_0_node")

    # Create a SMACH state machine
    sm_mission = smach.StateMachine(outcomes=['mission_finished'])

    with sm_mission:
        altitude = 0.5
        position = [[1.2, 0.0, altitude], [1.2, 0.0, 1.5], [0.0, 0.0, 1.5]]
        yaw = [180]

        sm_its_flying = smach.StateMachine(outcomes=["succeeded"])
        
        with sm_its_flying:

            con_wait_for_height = smach.Concurrence(outcomes=['wait_for_height','ready_to_nav'],
                                    default_outcome = 'wait_for_height',
                                    outcome_map={
                                        'ready_to_nav': {'TAKEOFF':'take_off','READ_HEIGHT':'ready'}
                                    })
            
            con_wait_for_position_1 = smach.Concurrence(outcomes=['wait_for_position_1','ready_to_yaw'],
                                    default_outcome = 'wait_for_position_1',
                                    outcome_map={
                                        'ready_to_yaw': {'NAVIGATION_1':'sailed','READ_POSITION_1':'ready'}
                                    })
            
            con_wait_for_yaw_1 = smach.Concurrence(outcomes=['wait_for_yaw_1','ready_to_nav_1'],
                                    default_outcome = 'wait_for_yaw_1',
                                    outcome_map={
                                        'ready_to_nav_1': {'YAW_1':'turned','READ_YAW_1':'ready'}
                                    })
            
            """
            con_wait_for_position_1 = smach.Concurrence(outcomes=['wait_for_position_1','ready_to_land'],
                                    default_outcome = 'wait_for_position_1',
                                    outcome_map={
                                        'ready_to_land': {'NAVIGATION_1':'sailed','READ_POSITION_1':'ready'}
                                    })
            
            con_wait_for_position_2 = smach.Concurrence(outcomes=['wait_for_position_2','ready_to_land'],
                                    default_outcome = 'wait_for_position_2',
                                    outcome_map={
                                        'ready_to_land': {'NAVIGATION_2':'sailed','READ_POSITION_2':'ready'}
                                    })
            
            con_wait_for_position_3 = smach.Concurrence(outcomes=['wait_for_position_3','ready_to_land'],
                                    default_outcome = 'wait_for_position_3',
                                    outcome_map={
                                        'ready_to_land': {'NAVIGATION_3':'sailed','READ_POSITION_3':'ready'}
                                    })
            """
            #######################################
            smach.StateMachine.add("ARMED", Armed(),
                                   transitions={
                                       "wait_for_arming":"ARMED",
                                        "armed" : "WAIT_FOR_HEIGHT"
                                    })
            
            with con_wait_for_height:
                smach.Concurrence.add('READ_HEIGHT', RangeFinderCheck(altitude))
                smach.Concurrence.add('TAKEOFF', TakeOff(altitude))
            
            smach.StateMachine.add("WAIT_FOR_HEIGHT", con_wait_for_height,
                                   transitions={
                                        'wait_for_height' : "WAIT_FOR_HEIGHT",
                                        'ready_to_nav' : "WAIT_FOR_POSITION_1"
                                    })
            
            with con_wait_for_position_1:
                smach.Concurrence.add('READ_POSITION_1', PositionCheck(target_position=position[0]))
                smach.Concurrence.add('NAVIGATION_1', Navigate(position[0]))

            
            smach.StateMachine.add("WAIT_FOR_POSITION_1", con_wait_for_position_1,
                                   transitions={
                                        'wait_for_position_1' : "WAIT_FOR_POSITION_1",
                                        'ready_to_yaw' : "WAIT_FOR_YAW_1"
                                    })
            
            with con_wait_for_yaw_1:
                smach.Concurrence.add('READ_YAW_1', YawCheck(target_yaw=yaw[0]))
                smach.Concurrence.add('YAW_1', Yaw(yaw[0]))

            
            smach.StateMachine.add("WAIT_FOR_YAW_1", con_wait_for_yaw_1,
                                   transitions={
                                        'wait_for_yaw_1' : "WAIT_FOR_YAW_1",
                                        'ready_to_nav_1' : "LAND"
                                    })
            
            """
            with con_wait_for_position_1:
                smach.Concurrence.add('READ_POSITION_1', PositionCheck(target_position=position[0]))
                smach.Concurrence.add('NAVIGATION_1', Navigate(position[0]))

            
            smach.StateMachine.add("WAIT_FOR_POSITION_1", con_wait_for_position_1,
                                   transitions={
                                        'wait_for_position_1' : "WAIT_FOR_POSITION_1",
                                        'ready_to_land' : "WAIT_FOR_POSITION_2"
                                    })
            
            with con_wait_for_position_2:
                smach.Concurrence.add('READ_POSITION_2', PositionCheck(target_position=position[1]))
                smach.Concurrence.add('NAVIGATION_2', Navigate(position[1]))

            
            smach.StateMachine.add("WAIT_FOR_POSITION_2", con_wait_for_position_2,
                                   transitions={
                                        'wait_for_position_2' : "WAIT_FOR_POSITION_2",
                                        'ready_to_land' : "WAIT_FOR_POSITION_3"
                                    })
            
            with con_wait_for_position_3:
                smach.Concurrence.add('READ_POSITION_3', PositionCheck(target_position=position[2]))
                smach.Concurrence.add('NAVIGATION_3', Navigate(position[2]))

            
            smach.StateMachine.add("WAIT_FOR_POSITION_3", con_wait_for_position_3,
                                   transitions={
                                        'wait_for_position_3' : "WAIT_FOR_POSITION_3",
                                        'ready_to_land' : "LAND"
                                    })
            """
            
        
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

