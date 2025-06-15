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
INHIBIT_PIN_0 = 11
INHIBIT_PIN_1 = 13
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
        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(INHIBIT_PIN_0, GPIO.IN)
        GPIO.setup(INHIBIT_PIN_1, GPIO.IN)

        smach.State.__init__(self, outcomes=['full_inhibit', 'partial_inhibit', 'no_inhibit'])
                

    def execute(self, userdata):
        # ret = inhibit_detect()

        for i in range(0, 10):
            self.p0 = GPIO.input(INHIBIT_PIN_0)
            rospy.sleep(0.05)
            if self.p0 == False:
                break

        for i in range(0, 10):
            self.p1 = GPIO.input(INHIBIT_PIN_1)
            rospy.sleep(0.05)
            if self.p0 == False:
                break

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

        GPIO.setmode(GPIO.BOARD) #TODO: Elias set board pin numbering mode
        GPIO.setup(START_SIGNAL_PIN, GPIO.IN)

        smach.State.__init__(self, outcomes=['start_mission', 'not_yet'])
                

    def execute(self, userdata):
        self.start = GPIO.input(START_SIGNAL_PIN)

        if self.start == 1:
            print("Passed Mission Check 1")
            rospy.sleep(0.1)
            self.start = GPIO.input(START_SIGNAL_PIN)

            if self.start == 1:
                print("Passed Mission Check 2")
                rospy.sleep(0.1)
                self.start = GPIO.input(START_SIGNAL_PIN)

                if self.start == 1:
                    print("Passed Mission Check 3")
                    rospy.sleep(0.1)
                    self.start = GPIO.input(START_SIGNAL_PIN)

                    if self.start == 1:
                        print("Passed Mission Check 4")
                        rospy.sleep(0.1)
                        self.start = GPIO.input(START_SIGNAL_PIN)
                        if self.start == 1:
                            print("Passed Mission Check 5")
                            rospy.sleep(0.1)
                            self.start = GPIO.input(START_SIGNAL_PIN)

                        else:
                            pass

                    else:
                        pass
                else:
                    pass
            else:
                pass
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
