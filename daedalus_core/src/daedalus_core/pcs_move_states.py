#! /usr/bin/python3

import rospy
import smach
import smach_ros

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


# Folding and unfolding
Unfold_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_FOLDING_STEPS = len(rospy.get_param('joints/folding'))

with Unfold_SM:
    for i in range(0, NUM_FOLDING_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)

        if i == NUM_FOLDING_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('folding/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('folding/' + step_str, allowed_attempts=2),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(1),
                    transitions={'Complete': 'step_' + str(i+1)})



Fold_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_REFOLDING_STEPS = len(rospy.get_param('joints/refolding'))

print(NUM_REFOLDING_STEPS)

with Fold_SM:
    for i in range(0, NUM_REFOLDING_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)

        if i == NUM_REFOLDING_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('refolding/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('refolding/' + step_str, allowed_attempts=2),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(1),
                    transitions={'Complete': 'step_' + str(i+1)})
