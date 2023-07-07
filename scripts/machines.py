import rospy
import smach
from smach import CBState
from typing import *
from check_states import *
from controller_states import *


def mission_start(target_height : float) -> smach.StateMachine:
    """
    Create the start machine to init flight

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    """
    sm = smach.StateMachine(outcomes=["succeeded"],
                            input_keys=["waypoint"],
                            output_keys=['waypoint'])

    with sm:
        smach.StateMachine.add("WAIT_ARMED",
                                Armed(),
                                transitions={
                                    "wait_for_arming": "WAIT_ARMED",
                                    "armed": "TAKEOFF"
                                })
        
        smach.StateMachine.add("TAKEOFF",
                                TakeOff(target_height=target_height),
                                transitions={
                                    "wait_for_autonomous_mode": "TAKEOFF",
                                    "wait_for_height": "CHECK_HEIGHT"
                                })
        
        smach.StateMachine.add("CHECK_HEIGHT",
                                RangeFinderCheck(target_height=target_height),
                                transitions={
                                    "wait_for_height": "CHECK_HEIGHT",
                                    "ready": "succeeded"
                                })
        
    return sm        
    
    
def goto(waypoint : List) -> smach.StateMachine:
    """
    Create the start machine navigate to waypoint

    Params:
    waypoint: position to move

    Returns:
    sm: the machine object machine 
    """

    sm = smach.StateMachine(outcomes=["succeeded"])

    with sm:
        
        smach.StateMachine.add("GO_TO", Navigate(position=waypoint),
                                   transitions={
                                        'wait_for_position' : "CHECK_POSITION"
                                    })
        
        smach.StateMachine.add("CHECK_POSITION", PositionCheck(target_position=waypoint),
                                   transitions={
                                        'wait_for_position' : "CHECK_POSITION",
                                        'ready' : "succeeded"
                                    })


    return sm

def goto_waypoint() -> smach.StateMachine:
    """
    Create the start machine navigate to waypoint

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    """

    sm = smach.StateMachine(outcomes=["succeeded"],
                            input_keys=["waypoint"],
                            output_keys=['waypoint'])

    with sm:
        
        smach.StateMachine.add("GO_TO", WaypointNavigate(),
                                transitions={
                                    'wait_for_waypoint' : "CHECK_POSITION"
                                })
        
        smach.StateMachine.add("CHECK_POSITION", WaypointCheck(),
                                transitions={
                                    'wait_for_waypoint' : "CHECK_POSITION",
                                    'ready' : "succeeded"
                                })


    return sm


def turne(turne : float = 0.0) -> smach.StateMachine:
    """
    Create the start machine navigate to waypoint

    Params:
    waypoint: position to move

    Returns:
    sm: the machine object machine 
    """

    sm = smach.StateMachine(outcomes=["succeeded"],
                            input_keys=["turne"],
                            output_keys=['turne'])
    
    sm.userdata.turne = turne

    with sm:
        
        smach.StateMachine.add("TURNE", Turnaround(),
                                transitions={
                                    "wait_for_turne" : "CHECK_TURNE"
                                })
        
        smach.StateMachine.add("CHECK_TURNE", TurnaroundCheck(),
                                transitions={
                                    "wait_for_turne" : "CHECK_TURNE",
                                    "ready" : "succeeded"
                                })


    return sm



        


