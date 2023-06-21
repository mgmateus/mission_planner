#!/usr/bin/env python3
"""This is an example of how to do mission planning using BaseController. Do not run this module!"""
import rospy
from base_states import *

if __name__ == "__main__":
    rospy.init_node("mission_plane_0_node")

    # Create a SMACH state machine
    mission = smach.StateMachine(outcomes=['mission_finished'])

    with mission:
        altitude = 1.0
        its_flying = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"],
                                        default_outcome="succeeded",
                                        input_keys=[],
                                        output_keys=[],
                                        child_termination_cb=None,
                                        outcome_cb=None)
        
        with its_flying:
            smach.StateMachine.add("ARMED", Armed(),
                                   transitions={
                                       "wait_for_arming":"ARMED",
                                        "armed" : "TAKEOFF"
                                    })
            
            smach.StateMachine.add("TAKEOFF", TakeOff(altitude),
                                   transitions={
                                       "wait_for_autonomous_mode":"TAKEOFF",
                                        "take_off" : "READY_TO_NAVIGATION"
                                    })
            
            smach.StateMachine.add("READY_TO_NAVIGATION", RangeFinderCheck(altitude),
                                   transitions={"wait_for_altitude":"READY_TO_NAVIGATION",
                                                "its_flying" : "ready"})
        
            smach.StateMachine.add("LAND", Land(),
                                   transitions={
                                        "land" : "succeeded"
                                    })
        
    
    # Execute SMACH plan
    outcome = mission.execute()

