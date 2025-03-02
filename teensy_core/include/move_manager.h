/**
 * Move manager
 */

 #include <ros/ros.h>
 #include "daedalus_msgs/joint_pose_cmd.h"
 #include "daedalus_msgs/teensy_message.h"

//  #define POSITION_ACCURACY
//  #define MAX_WAIT

class MoveManager
{
    public:
        MoveManager(ros::NodeHandle *nh);
        ros::ServiceServer joint_pose_service;
        ros::Publisher joint_position_cmd;


    private:
        bool joint_pose_cmd(daedalus_msgs::joint_pose_cmd::Request &req,
                            daedalus_msgs::joint_pose_cmd::Response &res);

        bool wait_until_complete(std::vector<double> joint_cmds);

};