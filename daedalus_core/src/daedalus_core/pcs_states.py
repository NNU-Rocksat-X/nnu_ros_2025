#! /usr/bin/python3

# import RPi.GPIO as GPIO

import rospy
import smach
import smach_ros

INHIBIT_PIN_0 = 12
INHIBIT_PIN_1 = 13

# TODO: remove the arm sync functionality

# 
class Check_Inhibit(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Full_Inhibit', 'Partial_Inhibit', 'No_Inhibit'])
        rospy.wait_for_service('full_inhibit_detection')
        

    def execute(self, userdata):
        time_elapsed = 0
        other_arm_param = other_arm + 'inhibit_status'
        inhibit_status = self.inhibit_status()

        rospy.set_param('inhibit_status', inhibit_status.state)

        # return information based on both inhibits
        if not inhibit_status.state:
            return 'Partial_Inhibit'
        elif not inhibit_status.state:
            return 'No_Inhibit'
        else:
            return 'Full_Inhibit'


class Wait_State(smach.State):
    """
    Waits for the specified wait_time where wait_time is in seconds
    """
    
    def __init__(self, wait_time):
        smach.State.__init__(self, outcomes=['Complete'])
        self.wait_time = wait_time

    def execute(self, userdata):
        rospy.sleep(self.wait_time)

        return 'Complete'