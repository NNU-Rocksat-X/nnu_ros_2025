/**
 * Move manager
 * 
 * This ROS node follows the standard multithreaded ROS node design where the 
 * services, publishers, and subscribers all share the threads to allow for 
 * optimal scheduling efficiency and minimal task downtime. 
 * 
 * @author Riley Mark 
 * @author March 18, 2025
 */

 #include <ros/ros.h>
 #include "daedalus_msgs/joint_pose_cmd.h"
 #include "daedalus_msgs/teensy_message.h"
 #include "teensy_comm.h"

//  #define POSITION_ACCURACY
//  #define MAX_WAIT

class MoveManager
{
    public:
        MoveManager(ros::NodeHandle *nh);
        ros::ServiceServer joint_pose_service;
        ros::Publisher joint_position_cmd;
        ros::Subscriber encoder_values;


    private:
        bool joint_pose_cmd (daedalus_msgs::joint_pose_cmd::Request &req,
                             daedalus_msgs::joint_pose_cmd::Response &res);

        void encoder_monitor (const daedalus_msgs::teensy_message::ConstPtr& msg);
        bool is_it_complete (void);

        bool wait_until_complete (std::vector<double> joint_cmds);


        volatile float desired_enc_pos[NUM_JOINTS];
        volatile float current_enc_pos[NUM_JOINTS];
        std::vector<double> move_tolerance;
        float move_timeout;

        std::string current_name_space;

};