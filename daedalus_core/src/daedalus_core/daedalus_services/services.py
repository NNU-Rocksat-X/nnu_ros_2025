
import rospy

from daedalus_msgs.srv import joint_pose_cmd

print('waiting for services')

rospy.wait_for_service('joint_pose_cmd')
joint_pose_cmd = rospy.ServiceProxy('joint_pose_cmd', joint_pose_cmd)
print("joint_pose_cmd found")