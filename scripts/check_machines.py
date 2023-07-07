import rospy
import smach
import time

from typing import *
from check_states import *
from controller_states import *
from machines import *


def check_flight(target_height : float) -> smach.StateMachine:
    """
    Create the machine to check drone in flight

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    """
    sm = smach.StateMachine(outcomes=["succeeded"])

    with sm:
        sm_mission_start = mission_start(target_height)

        
        
        smach.StateMachine.add("FLYING", sm_mission_start,
                                   transitions={
                                        "succeeded" : "LAND"
                                    })

        smach.StateMachine.add("LAND",
                               Land(),
                               transitions={
                                   "land": "succeeded"
                               })
        
    return sm

def check_navigation(target_height : float, waypoint : List) -> smach.StateMachine:
    """
    Create the machine to test navigation with discreted waypoint

    Params:
    target_height: altitude to take off
    waypoint: waypoint as [x, y, z]

    Returns:
    sm: the machine object machine 
    """

    sm = smach.StateMachine(outcomes=["succeeded"])

    check_position = waypoint
    check_position[0] = check_position[0] + 0.25

    with sm:
        sm_mission_start = mission_start(target_height)

        smach.StateMachine.add("FLYING", sm_mission_start,
                                   transitions={
                                        "succeeded" : "GO_TO"
                                    })
        
        smach.StateMachine.add("GO_TO", Navigate(position=waypoint),
                                   transitions={
                                        'wait_for_position' : "CHECK_POSITION"
                                    })
        
        smach.StateMachine.add("CHECK_POSITION", PositionCheck(target_position=waypoint),
                                   transitions={
                                        'wait_for_position' : "CHECK_POSITION",
                                        'ready' : "CHECK_END"
                                    })
        
        smach.StateMachine.add("CHECK_END", PositionCheck(target_position=check_position),
                                   transitions={
                                        'wait_for_position' : "GO_TO",
                                        'ready' : "LAND"
                                    })
        
        smach.StateMachine.add("LAND",
                               Land(),
                               transitions={
                                   "land": "succeeded"
                               })
        
    return sm

def check_waypoints_navigation_error(target_height : float, waypoints : List) -> smach.StateMachine:
    """
    Create the machine to test navigation with waypoints

    Params:
    target_height: altitude to take off
    waypoints: waypoints as [x, y, z]
    
    Returns:
    sm: the machine object machine 
    """

    sm = smach.StateMachine(outcomes=["succeeded"])
    with sm:
        sm_mission_start = mission_start(target_height)
        
        smach.StateMachine.add("FLYING", sm_mission_start,
                                transitions={
                                    "succeeded" : "GO_TO_WAIPOINTS"
                                })
        
        it_goto = smach.Iterator(outcomes = ['succeeded'],
                                    input_keys = ['waypoints', 'waypoint'],
                                    output_keys=['waypoints', 'waypoint'],
                                    it = lambda: range(0, len(waypoints)),
                                    it_label = 'i',
                                    exhausted_outcome = 'succeeded')
        
        with it_goto:
            container_sm = smach.StateMachine(outcomes = ['succeeded','continue'],
                                            input_keys = ['waypoints', 'waypoint'],
                                            output_keys = ['waypoints', 'waypoint'])
            
            container_sm.userdata.waypoints = waypoints
            container_sm.userdata.waypoint = container_sm.userdata.waypoints.pop(0)

            with container_sm:

                @smach.cb_interface(input_keys=['waypoints', 'waypoint'],
                                    output_keys=['waypoints', 'waypoint'], 
                                    outcomes=['succeeded'])
                
                def pop_waypoint_cb(userdata):
                    userdata.waypoint = userdata.waypoints.pop(0)
                    return 'succeeded'

                smach.StateMachine.add('POP_WAIPOINT', smach.CBState(pop_waypoint_cb), 
                                {'succeeded':'GO_TO_WAIPOINT'})
                
                
                smach.StateMachine.add("GO_TO_WAIPOINT", Navigate(position=container_sm.userdata.waypoint),
                                   transitions={
                                        'wait_for_position' : "CHECK_POSITION"
                                    })
        
                smach.StateMachine.add("CHECK_POSITION", PositionCheck(target_position=container_sm.userdata.waypoint),
                                        transitions={
                                                'wait_for_position' : "CHECK_POSITION",
                                                'ready' : "CHECK_WAIPOINTS"
                                            })
                
                @smach.cb_interface(input_keys=['waypoints'],   
                                    outcomes=['succeeded', 'continue'])
                
                def finished_waypoints_cb(userdata):
                    return 'succeeded' if len(userdata.waypoints) == 0 else 'continue'
                
                smach.StateMachine.add('CHECK_WAIPOINTS', smach.CBState(finished_waypoints_cb), 
                                {'succeeded':'succeeded',
                                'continue':'continue'
                                })

            
            #close container_sm
            smach.Iterator.set_contained_state('CONTAINER_STATE', 
                                        container_sm, 
                                        loop_outcomes=['continue'])
        #close it_goto        
        smach.StateMachine.add("GO_TO_WAIPOINTS", it_goto,
                                transitions={
                                    "succeeded" : "LAND"
                                })
        
        smach.StateMachine.add("LAND",
                               Land(),
                               transitions={
                                   "land": "succeeded"
                               })
        
    return sm 


def check_waypoints_navigation(target_height : float, waypoints : List) -> smach.StateMachine:
    """
    Create the machine to test navigation with waypoints

    Params:
    target_height: altitude to take off
    waypoints: waypoints as [x, y, z]
    
    Returns:
    sm: the machine object machine 
    """

    sm = smach.StateMachine(outcomes=["succeeded"])
    sm.userdata.waypoints = waypoints
    sm.userdata.waypoint = None #sm.userdata.waypoints.pop(0)

    with sm:
        sm_mission_start = mission_start(target_height)
        
        smach.StateMachine.add("FLYING", sm_mission_start,
                                transitions={
                                    "succeeded" : "POP_WAIPOINT"
                                })
        @smach.cb_interface(input_keys=['waypoints', 'waypoint'],
                            output_keys=['waypoints', 'waypoint'], 
                            outcomes=['succeeded'])
                
        def pop_waypoint_cb(userdata):
            userdata.waypoint = userdata.waypoints.pop(0)
            return 'succeeded'

        smach.StateMachine.add('POP_WAIPOINT', smach.CBState(pop_waypoint_cb), 
                        {'succeeded':'GO_TO_WAIPOINT'})
        
        sm_goto_waypoint = goto_waypoint()

        smach.StateMachine.add("GO_TO_WAIPOINT", sm_goto_waypoint,
                                transitions={
                                    "succeeded" : "CHECK_WAIPOINTS"
                                })
        
        @smach.cb_interface(input_keys=['waypoints'],   
                            outcomes=['succeeded', 'continue'])
        
        def finished_waypoints_cb(userdata):
            return 'succeeded' if len(userdata.waypoints) == 0 else 'continue'
        
        smach.StateMachine.add('CHECK_WAIPOINTS', smach.CBState(finished_waypoints_cb), 
                        {'succeeded':'LAND',
                         'continue':'POP_WAIPOINT'
                        })
        
        
        smach.StateMachine.add("LAND", Land(),
                               transitions={
                                   "land": "succeeded"
                               })
        
    return sm 


def check_turne(target_height : float, target_turne : float) -> smach.StateMachine:
    """
    Create the machine to test navigation with waypoints

    Params:
    target_height: altitude to take off
    waypoints: waypoints as [x, y, z]
    
    Returns:
    sm: the machine object machine 
    """

    sm = smach.StateMachine(outcomes=["succeeded"])
    
    sm.userdata.turne = target_turne
    

    with sm:
        sm_mission_start = mission_start(target_height)

        smach.StateMachine.add("FLYING", sm_mission_start,
                                transitions={
                                    "succeeded" : "GO_TO_WAIPOINT"
                                })
        
        sm_goto_waypoint = goto_waypoint()

        smach.StateMachine.add("GO_TO_WAIPOINT", sm_goto_waypoint,
                                transitions={
                                    "succeeded" : "TURNE"
                                })
    
        
        smach.StateMachine.add("TURNE", Turnaround(),
                                transitions={
                                    "wait_for_turne" : "CHECK_TURNE"
                                })
        
        smach.StateMachine.add("CHECK_TURNE", TurnaroundCheck(),
                                transitions={
                                    "wait_for_turne" : "CHECK_TURNE",
                                    "ready" : "succeeded"
                                })
        
        smach.StateMachine.add("STABILIZE_WAIPOINT", sm_goto_waypoint,
                                transitions={
                                    "succeeded" : "succeeded"
                                })
        
        #smach.StateMachine.add("LAND", Land(),
        #                       transitions={
        #                           "land": "succeeded"
        #                       })
        
    return sm