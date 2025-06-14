#! /usr/bin/python3

#
# PCS Move States
#
# @brief sub states for the mission state machine. This file contains all
#        non-movement related state machines and helper functions. 
#
# @author Riley Mark 
# @author February 27, 2025
# 
#*******************************************************************************

import rospy
import smach
import smach_ros
import Jetson.GPIO as GPIO
from daedalus_core.daedalus_services.services import *

# UNUSED
# INHIBIT_PIN_0 = 11
# INHIBIT_PIN_1 = 13
START_SIGNAL_PIN = 15

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

        # TODO: if this does not work, lets just do a quick poll instead of this
        #       overly complex service thing...
        # steps are: 
        # init gpio into the correct mode (input)
        # wait a few ms or like a second if time isnt an issue
        # check if its high or low
        # wait a few ms
        # check again
        # if its still high, return inhibited (or not depending on your electrical system)
        # if its not still high, its not actually inhibited and the first reading was a false positive
        # 
        # so boom there you go. This should be a super quick exit strategy for if
        # the legacy code does not work.


        self.p0 = ret.inhibit_0_status
        self.p1 = ret.inhibit_1_status

        if self.p0 and self.p1:
            return 'full_inhibit'
        elif self.p0 or self.p1:
            return 'partial_inhibit'
        else:
            return 'no_inhibit'


##
# 
##
class Check_Signal(smach.State):
    def __init__(self):
        self.start = False

        GPIO.setmode(GPIO.BCM) #TODO: Elias set board pin numbering mode
        GPIO.setup(START_SIGNAL_PIN, GPIO.IN)

        smach.State.__init__(self, outcomes=['start_mission', 'not_yet'])
                

    def execute(self, userdata):
        self.start = GPIO.input(START_SIGNAL_PIN)

        if self.start == 1:
            rospy.sleep(0.1)
            self.start = GPIO.input(START_SIGNAL_PIN)
        else:
            pass

        if self.start:
            return 'start_mission'
        else:
            return 'not_yet'


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
