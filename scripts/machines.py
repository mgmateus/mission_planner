import rospy
import smach
from smach import CBState
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
    
def goto(target_height : float, position : List) -> smach.StateMachine:

    sm = smach.StateMachine(outcomes=["succeeded"])

    with sm:
        sm_mission_start = mission_start(target_height)

        smach.StateMachine.add("FLYING", sm_mission_start,
                                   transitions={
                                        "succeeded" : "GO_TO"
                                    })
        
        smach.StateMachine.add("GO_TO", Navigate(position=position),
                                   transitions={
                                        'wait_for_position' : "CHECK_POSITION"
                                    })
        
        smach.StateMachine.add("CHECK_POSITION", PositionCheck(target_position=position),
                                   transitions={
                                        'wait_for_position' : "CHECK_POSITION",
                                        'ready' : "succeeded"
                                    })


    return sm
        
def test_nav(target_height : float, position : List) -> smach.StateMachine:

    sm = smach.StateMachine(outcomes=["succeeded"])

    check_position = position
    check_position[0] = check_position[0] + 0.25

    with sm:
        sm_mission_start = mission_start(target_height)

        smach.StateMachine.add("FLYING", sm_mission_start,
                                   transitions={
                                        "succeeded" : "GO_TO"
                                    })
        
        smach.StateMachine.add("GO_TO", Navigate(position=position),
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

def test_altitude_ajust(target_height : float, path : List) -> smach.StateMachine:

    sm = smach.StateMachine(outcomes=["succeeded"])
    sm.userdata.path = path

    with sm:
        sm_mission_start = mission_start(target_height)
        
        smach.StateMachine.add("FLYING", sm_mission_start,
                                transitions={
                                    "succeeded" : "GO_TO_PATH"
                                })
        
        it_goto = smach.Iterator(outcomes = ['succeeded'],
                                    input_keys = ['path'],
                                    output_keys=['position'],
                                    it = lambda: range(0, len(path)),
                                    it_label = 'i',
                                    exhausted_outcome = 'succeeded')
        
        with it_goto:
            container_sm = smach.StateMachine(outcomes = ['succeeded','continue'],
                                            input_keys = ['path'],
                                            output_keys = ['position'])
            
            with container_sm:

                @smach.cb_interface(input_keys=['path'],
                                    output_keys=['position'], 
                                    outcomes=['succeeded'])
                
                def pop_position_cb(ud):
                    ud.position = ud.path.pop(0)
                    return 'succeeded'

                smach.StateMachine.add('POP_POSITION', CBState(pop_position_cb), 
                                {'succeeded':'GO_TO'})
                
                sm_goto = goto(target_height, sm.userdata.position)
                
                smach.StateMachine.add("GO_TO", sm_goto,
                                    transitions={
                                        "succeeded" : "CHECK_PATH"
                                    })
                
                @smach.cb_interface(input_keys=['path'],
                                    output_keys=['path'], 
                                    outcomes=['succeeded', 'continue'])
                
                def finished_path_cb(ud):
                    return 'succeeded' if len(ud.path) == 0 else 'continue'
                
                smach.StateMachine.add('CHECK_PATH', CBState(finished_path_cb), 
                                {'succeeded':'succeeded',
                                'continue':'continue'
                                })

            
            #close container_sm
            Iterator.set_contained_state('CONTAINER_STATE', 
                                        container_sm, 
                                        loop_outcomes=['continue'])
        #close it_goto        
        smach.StateMachine.add("GO_TO_PATH", it_goto,
                                transitions={
                                    "succeeded" : "LAND"
                                })
        
        smach.StateMachine.add("LAND",
                               Land(),
                               transitions={
                                   "land": "succeeded"
                               })
        
    return sm 