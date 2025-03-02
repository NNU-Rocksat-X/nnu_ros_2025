
/**
 * Move manager
 * 
 * TODO: set up subscriber to publisher for encoder values feedback
 */
#include "move_manager.h"

/**
 * 
 */
MoveManager::MoveManager (ros::NodeHandle *nh)
{
    joint_pose_service = nh->advertiseService("joint_pose_cmd", 
                                            &MoveManager::joint_pose_cmd, this);

    joint_position_cmd = nh->advertise<daedalus_msgs::teensy_message>("update_teensy_cmd", 10, this);

    ROS_INFO("initialized the Move Manager");

}

/**
 * 
 */
bool MoveManager::joint_pose_cmd (daedalus_msgs::joint_pose_cmd::Request &req,
                                  daedalus_msgs::joint_pose_cmd::Response &res)
{
    daedalus_msgs::teensy_message tnsy_msg;
    std::string param;

    param =  "/daedalus/" + req.pose_name;
    ROS_INFO("Moving to joint state: %s", param.c_str());
    
    std::vector<double> joint_group_positions; // changed from double to float
    
    if (ros::param::get(param, joint_group_positions))
    {
        ROS_INFO("Joint angles: %f, %f, %f", joint_group_positions[0], joint_group_positions[1], joint_group_positions[2]);
    
        bool plan_success = true;
    
        if (plan_success) {
            
        
            for (int kk = 0; kk < joint_group_positions.size(); ++kk) { 
                tnsy_msg.steps.push_back(joint_group_positions[kk]);
            }
    
            ROS_INFO("publishing");
            joint_position_cmd.publish(tnsy_msg);
    
            ros::Duration(0.1).sleep();
    
            // bool completion_status = wait_until_complete(joint_group_positions);
            res.done = true;
            return true;
        }
        else 
        {
            res.done = false;
            return true;
        }
    } 
    else 
    {
        res.done = false;
        ROS_WARN("Joint pose does not exist!");
        
        return true;
    }
}

/**
 * 
 */
bool MoveManager::wait_until_complete(std::vector<double> joint_cmds)
{
    float POSITION_ACCURACY = 0.1;
    int MAX_WAIT = 100;
    std::vector<std::string> joint_names;

    ros::param::get("stepper_config/joint_names", joint_names);
    ros::param::get("stepper_config/goal_position_accuracy", POSITION_ACCURACY);
    ros::param::get("stepper_config/max_goal_time", MAX_WAIT);

    int joints_complete = 0;
    int cnt = 0;

    while (joints_complete < joint_cmds.size())
    {
        ROS_INFO("Waiting for move to finish...");
        joints_complete = 0;
        // robot_state::RobotState current_state(*move_group->getCurrentState());
        for (int i = 0; i < 8; i++) // removed size of joint_cmds to replace it with 7.. trying to remove hang time from this function
        {
            // const double* joint_pos = current_state.getJointPositions(joint_names[i]);
            // ROS_INFO("---- J%i ----", i);
            // ROS_INFO("Current: %f", *joint_pos);
            // ROS_INFO("cmd: %f", joint_cmds[i]);
            // if (abs(*joint_pos - joint_cmds[i]) < POSITION_ACCURACY) {
            //     joints_complete += 1;
            // }
        }

        if (cnt > MAX_WAIT*10)
        {
            return false;
        }
        cnt++;
        ros::Duration(0.1).sleep();
    }
    return true;
}



/**
 * M A I N
 */
int main (int argc, char** argv)
{
    ros::init(argc, argv, "move_manager");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    spinner.start();

    MoveManager move_manager = MoveManager(&nh);
    ros::waitForShutdown();
}