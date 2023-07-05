#!/usr/bin/env python3
"""This is an example of how to do mission planning using BaseController. Do not run this module!"""
import rospy
from machines import pilot_flight, test_nav

if __name__ == "__main__":
    rospy.init_node("mission_plane_0_node")

    # Create a SMACH state machine
    height = 0.6
    target = [0.25, 0.0, 0.6]
    sm_mission = test_nav(height, target)
    
    # Execute SMACH plan
    outcome = sm_mission.execute()

