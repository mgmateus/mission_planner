import rospy
import smach
from typing import *
from check_states import *
from states import *


def mission_start(target_height : float) -> smach.StateMachine:
    """
    Create the start machine to init flight

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    """
    sm = smach.StateMachine(outcomes=["succeeded"])

    with sm:
        rospy.logwarn('MISSION_START')
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
        
    
def pilot_flight(target_height : float) -> smach.StateMachine:
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
        
        smach.StateMachine.add("FLING", sm_mission_start,
                                   transitions={
                                        "succeeded" : "LAND"
                                    })

        smach.StateMachine.add("LAND",
                               Land(),
                               transitions={
                                   "land": "succeeded"
                               })
    
def goto(target_height : float, position : List) -> smach.StateMachine:

    sm = smach.StateMachine(outcomes=["succeeded"])

    with sm:
        sm_mission_start = mission_start(target_height)

        smach.StateMachine.add("FLING", sm_mission_start,
                                   transitions={
                                        "succeeded" : "GO_TO"
                                    })
        
        smach.StateMachine.add("GO_TO", Navigate(position=position),
                                   transitions={
                                        'wait_for_position' : "GO_TO"
                                    })
        
        smach.StateMachine.add("CHECK_POSITION", PositionCheck(target_position=position),
                                   transitions={
                                        'wait_for_position' : "CHECK_POSITION",
                                        'ready' : "succeeded"
                                    })
        
def test_nav(target_height : float, position : List) -> smach.StateMachine:

    sm = smach.StateMachine(outcomes=["succeeded"])

    with sm:
        sm_mission_start = mission_start(target_height)

        smach.StateMachine.add("FLING", sm_mission_start,
                                   transitions={
                                        "succeeded" : "GO_TO"
                                    })
        
        smach.StateMachine.add("GO_TO", Navigate(position=position),
                                   transitions={
                                        'wait_for_position' : "GO_TO"
                                    })
        
        smach.StateMachine.add("CHECK_POSITION", PositionCheck(target_position=position),
                                   transitions={
                                        'wait_for_position' : "CHECK_POSITION",
                                        'ready' : "CHECK_END"
                                    })
        
        smach.StateMachine.add("CHECK_END", PositionCheck(target_position=position[0] + 0.4),
                                   transitions={
                                        'wait_for_position' : "GO_TO",
                                        'ready' : "succeeded"
                                    })