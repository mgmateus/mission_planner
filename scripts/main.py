#!/usr/bin/env python3
"""This is an example of how to do mission planning using BaseController. Do not run this module!"""
import rospy
from base_states import Armed, TakeOff, RangeFinderCheck

if __name__ == "__main__":
    rospy.init_node("mission_plane_0_node")

    # Create a SMACH state machine
    mission = smach.StateMachine(outcomes=['mission_finished'])

    with mission:
        listenner = smach.StateMachine(outcomes=['listening'])
        controller = smach.StateMachine(outcomes=['position_hold'])
    
        with listenner:
            smach.StateMachine.add('Ready_Altitude', RangeFinderCheck(1.0),
                                   transitions={'ready_altitude':'listening'})
        # Open the container
        with controller:
            # Add states to the container
            smach.StateMachine.add('Armed', Armed(), 
                                   transitions={'armed':'TakeOff'})

            smach.StateMachine.add('TakeOff', TakeOff(1.0), 
                                   transitions={'takeoff':'position_hold'})
    
    # Execute SMACH plan
    outcome = mission.execute()

