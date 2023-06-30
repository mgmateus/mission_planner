#!/usr/bin/env python3
"""This is an example of how to do mission planning using BaseController. Do not run this module!"""
import rospy
import smach
from states import *

if __name__ == "__main__":
    rospy.init_node("mission_plane_0_node")

    sm = smach.StateMachine(outcomes=["succeeded"])
    with sm:
        smach.StateMachine.add("WAIT_ARMED",
                               Armed(),
                               transitions={
                                   "wait_for_arming": "WAIT_ARMED",
                                   "armed": "TAKEOFF"
                               })
        smach.StateMachine.add("TAKEOFF",
                               TakeOff(target_height=1.0),
                               transitions={
                                   "wait_for_autonomous_mode": "TAKEOFF",
                                   "wait_for_height": "CHECK_HEIGHT",
                                   "take_off": "TAKEOFF"
                               })
        smach.StateMachine.add("CHECK_HEIGHT",
                               RangeFinderCheck(target_height=1.0),
                               transitions={
                                   "wait_for_height": "CHECK_HEIGHT",
                                   "ready": "CALL_LAND"
                               })
        smach.StateMachine.add("CALL_LAND",
                               Land(),
                               transitions={
                                   "land": "succeeded"
                               })

    outcomes = sm.execute()
