
import rospy

from daedalus_msgs.srv import joint_pose_cmd
from daedalus_msgs.srv import inhibit_detection

rospy.loginfo('waiting for services')

rospy.wait_for_service('joint_pose_cmd')
joint_pose_cmd = rospy.ServiceProxy('joint_pose_cmd', joint_pose_cmd)
rospy.loginfo("joint_pose_cmd found")

#rospy.wait_for_service('inhibit_detection_0')
#inhibit_detect = rospy.ServiceProxy('inhibit_detection_0', inhibit_detection)
#rospy.loginfo("inhibit_detection_0 found")
