import rospy
import smach
from smach import CBState
from typing import *
from check_states import *
from states import *
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
        
        smach.StateMachine.add("CHECK_POSITION", PositionCheck(target_position=position),
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
    sm.userdata.waypoint = None
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
            
            with container_sm:

                @smach.cb_interface(input_keys=['waypoints', 'waypoint'],
                                    output_keys=['waypoints', 'waypoint'], 
                                    outcomes=['succeeded'])
                
                def pop_waypoint_cb(ud):
                    ud.waypoint = ud.waypoints.pop(0)
                    return 'succeeded'

                smach.StateMachine.add('POP_WAIPOINT', CBState(pop_waypoint_cb), 
                                {'succeeded':'GO_TO'})
                
                sm_goto = goto(target_height, sm.userdata.waypoint)
                
                smach.StateMachine.add("GO_TO", sm_goto,
                                    transitions={
                                        "succeeded" : "CHECK_WAIPOINTS"
                                    })
                
                @smach.cb_interface(input_keys=['waypoints'],   
                                    outcomes=['succeeded', 'continue'])
                
                def finished_waypoints_cb(ud):
                    return 'succeeded' if len(ud.waypoints) == 0 else 'continue'
                
                smach.StateMachine.add('CHECK_WAIPOINTS', CBState(finished_waypoints_cb), 
                                {'succeeded':'succeeded',
                                'continue':'continue'
                                })

            
            #close container_sm
            Iterator.set_contained_state('CONTAINER_STATE', 
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