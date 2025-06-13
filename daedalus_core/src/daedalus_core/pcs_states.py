#! /usr/bin/python3

#
# PCS Move States
#
# @brief sub states for the mission state machine. This file contains all
#        non-movement related state machines and helper functions. 
#
# @author Riley Mark 
# @author Febuary 27, 2025
# 
#*******************************************************************************

import rospy
import smach
import smach_ros
from daedalus_core.daedalus_services.services import *

INHIBIT_PIN_0 = 11
INHIBIT_PIN_1 = 13
MISSION_GO_PIN = 15

##
# Calls the inhibit detection service. Checks 2 pins and sets the inhibit state 
# based on the quantity of pins set to inhibit state. 
#
# Inhibit pin numbers set in the detector node.
##
class Check_Inhibit(smach.State):
    def __init__(self):
        self.p0 = False
        self.p1 = False
        smach.State.__init__(self, outcomes=['full_inhibit', 'partial_inhibit', 'no_inhibit'])
                

    def execute(self, userdata):
        ret = inhibit_detect()

        #self.p0 = False
        #self.p1 = False

        self.p0 = ret.inhibit_0_status
        self.p1 = ret.inhibit_1_status

        if self.p0 and self.p1:
            return 'full_inhibit'
        elif self.p0 or self.p1:
            return 'partial_inhibit'
        else:
            return 'no_inhibit'


##
# Wait! State machine state for 
#
# @param wait_time - time to wait in seconds
##
class Wait_State(smach.State):
    def __init__(self, wait_time):
        smach.State.__init__(self, outcomes=['Complete'])
        self.wait_time = wait_time


    def execute(self, userdata):
        rospy.sleep(self.wait_time)

        return 'Complete'
