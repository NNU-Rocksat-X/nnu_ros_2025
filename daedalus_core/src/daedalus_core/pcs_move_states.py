#!/usr/bin/env python

#
# PCS Move States
#
# @brief sub states for the mission state machine. This file contains all
#        movement related state machines. 
#
# @author Riley Mark 
# @author March 14, 2025
# 
#*******************************************************************************

import rospy
import smach
import smach_ros

from daedalus_core.pcs_states import *
from daedalus_core.daedalus_services.services import *


class Joint_Pose_State(smach.State):
    
#Takes pose name as parameter and executes, transitions to next state when done.
    """
    USAGE EXAMPLE
        smach.StateMachine.add('Pre_Throw', Joint_Pose_State("pre_throw"),
                  transitions={'Success': 'Jetson_Sync_1',
                               'Fail': 'Ball_Status'})
    """
    def __init__(self, pose, allowed_attempts=1):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])
        self.pose = pose
        self.allowed_attempts = allowed_attempts

    def execute(self, userdata):
        complete = False
        attempts = 0

        while not complete and attempts < self.allowed_attempts:
            #joint_pose_cmd(self.pose)
            complete = joint_pose_cmd(self.pose).done

        if complete:
            return 'Success'
        else:
            return 'Fail'

##
# Folding and Unfolding State Machines
#
# The unfolding state machine iterates through the unfold positions listed in
# config/arm_positions.yaml. The folding state machine iterates through the same
# positions, but backwards
##
unfold_sm = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_FOLDING_STEPS = len(rospy.get_param('unfold'))
with unfold_sm:
    for i in range(0, NUM_FOLDING_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)


        if i == NUM_FOLDING_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('unfold/' + step_str, allowed_attempts=1),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            rospy.loginfo(i)
            smach.StateMachine.add(step_str, Joint_Pose_State('unfold/' + step_str, allowed_attempts=1),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(3),
                                   transitions={'Complete': 'step_' + str(i + 1)})

fold_sm = smach.StateMachine(outcomes=['Success', 'Fail'])
with fold_sm:
    for i in range(NUM_FOLDING_STEPS - 1, -1, -1):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)

        if i == 0:
            print("last folding step")
            smach.StateMachine.add(step_str, Joint_Pose_State('unfold/' + step_str, allowed_attempts=1),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('unfold/' + step_str, allowed_attempts=1),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(3),
                    transitions={'Complete': 'step_' + str(i - 1)})


poses_sm = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_POSE_STEPS = len(rospy.get_param('pose'))
with poses_sm:
    for i in range(0, NUM_POSE_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)


        if i == NUM_FOLDING_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('pose/' + step_str, allowed_attempts=1),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            rospy.loginfo(i)
            smach.StateMachine.add(step_str, Joint_Pose_State('pose/' + step_str, allowed_attempts=1),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(3),
                                   transitions={'Complete': 'step_' + str(i + 1)})

poses_sm = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_POSE_STEPS = len(rospy.get_param('pose'))
with poses_sm:
    for i in range(0, NUM_POSE_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)


        if i == NUM_POSE_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('pose/' + step_str, allowed_attempts=1),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            rospy.loginfo(i)
            smach.StateMachine.add(step_str, Joint_Pose_State('pose/' + step_str, allowed_attempts=1),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(3),
                                   transitions={'Complete': 'step_' + str(i + 1)})

ejector_sm = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_EJECT_STEPS = len(rospy.get_param('ejector'))
with ejector_sm:
    for i in range(0, NUM_EJECT_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)


        if i == NUM_EJECT_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('ejector/' + step_str, allowed_attempts=1),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            rospy.loginfo(i)
            smach.StateMachine.add(step_str, Joint_Pose_State('ejector/' + step_str, allowed_attempts=1),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(3),
                                   transitions={'Complete': 'step_' + str(i + 1)})


ejector_sm_two = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_EJECT_TWO_STEPS = len(rospy.get_param('second_ejector'))
with ejector_sm_two:
    for i in range(0, NUM_EJECT_TWO_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)


        if i == NUM_EJECT_TWO_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('second_ejector/' + step_str, allowed_attempts=1),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            rospy.loginfo(i)
            smach.StateMachine.add(step_str, Joint_Pose_State('second_ejector/' + step_str, allowed_attempts=1),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(0.1),
                                   transitions={'Complete': 'step_' + str(i + 1)})


ejector_sm_three = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_EJECT_THREE_STEPS = len(rospy.get_param('third_ejector'))
with ejector_sm_three:
    for i in range(0, NUM_EJECT_THREE_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)


        if i == NUM_EJECT_THREE_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('third_ejector/' + step_str, allowed_attempts=1),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            rospy.loginfo(i)
            smach.StateMachine.add(step_str, Joint_Pose_State('third_ejector/' + step_str, allowed_attempts=1),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(0.1),
                                   transitions={'Complete': 'step_' + str(i + 1)})



ejector_sm_four = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_EJECT_FOUR_STEPS = len(rospy.get_param('fourth_ejector'))
with ejector_sm_four:
    for i in range(0, NUM_EJECT_FOUR_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)


        if i == NUM_EJECT_FOUR_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('third_ejector/' + step_str, allowed_attempts=1),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            rospy.loginfo(i)
            smach.StateMachine.add(step_str, Joint_Pose_State('third_ejector/' + step_str, allowed_attempts=1),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(0.1),
                                   transitions={'Complete': 'step_' + str(i + 1)})
