#!/usr/bin/env python3
"""This is an example of how to do mission planning using BaseController. Do not run this module!"""
import rospy
from machines import mission_start
from check_machines import check_flight, check_navigation, check_waypoints_navigation

if __name__ == "__main__":
    rospy.init_node("mission_plane_0_node")

    # Create a SMACH state machine
    height = 0.6
    target = [0.25, 0.0, 0.6]
    waypoints = [[1.2, 0.0, height], [1.2, 0.0, 1.5], [0.0, 0.0, 1.5]]

    sm_mission = mission_start(height) #take_off maroto sem land

    #sm_mission = check_flight(1.0) #take_off maroto

    #sm_mission = check_waypoints_navigation(height, waypoints) #teste do rangefinder


    
    # Execute SMACH plan
    outcome = sm_mission.execute()

