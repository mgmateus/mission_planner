#!/usr/bin/env python3
"""This is an example of how to do mission planning using BaseController. Do not run this module!"""
import rospy
from machines import pilot_flight, test_nav

if __name__ == "__main__":
    rospy.init_node("mission_plane_0_node")

    # Create a SMACH state machine
    sm_mission = test_nav(1.0, [0.1, 0.0, 0.8])
    
    # Execute SMACH plan
    outcome = sm_mission.execute()

