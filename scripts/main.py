#!/usr/bin/env python3
"""This is an example of how to do mission planning using BaseController. Do not run this module!"""
import rospy
from base_states import Armed, TakeOff

if __name__ == "__main__":
    rospy.init_node("mission_plane_0_node")

    # Create a SMACH state machine
    controller = smach.StateMachine(outcomes=['flighting'])
    

    # Open the container
    with controller:
        # Add states to the container
        smach.StateMachine.add('Armed', Armed(), 
                               transitions={'armed':'TakeOff'})

        smach.StateMachine.add('TakeOff', TakeOff(), 
                               transitions={'takeoff':'flighting'})
    
    # Execute SMACH plan
    outcome = controller.execute()

