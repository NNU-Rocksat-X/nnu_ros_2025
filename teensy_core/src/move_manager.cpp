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

#include "move_manager.h"

/**
 * The MoveManager constructor
 * 
 * Initializes the desired encoder and current encoder arrays. Grabs movement 
 * parameters from ROS params. Initializes the joint pose service, joint 
 * position command publisher, and the encoder values subscriber.
 * 
 * @return - none
 */
MoveManager::MoveManager (ros::NodeHandle *nh)
{
    std::string timeout_param_key;
    std::string move_tolerance_param_key;

    for (int ii = 0; ii < NUM_JOINTS; ii++)
    {
        desired_enc_pos[ii] = 200;
        current_enc_pos[ii] = 0;
        //move_tolerance[ii] = 0.1;
    }
    
    current_name_space = ros::this_node::getNamespace();
    timeout_param_key = current_name_space + "/movement/timeout";
    move_tolerance_param_key = current_name_space + "/movement/tolerance";


    if (!ros::param::get(timeout_param_key, move_timeout))
    {
        ROS_WARN("timeout param not found");
    }

    if (!ros::param::get(move_tolerance_param_key, move_tolerance))
    {
        ROS_WARN("tolerance param not found");
    }

    joint_pose_service = nh->advertiseService("joint_pose_cmd", 
                                              &MoveManager::joint_pose_cmd, 
                                              this);

    joint_position_cmd = nh->advertise<daedalus_msgs::teensy_message>("update_teensy_cmd", 
                                                                      10, 
                                                                      this);

    encoder_values = nh->subscribe("encoder_values", 
                                   10, 
                                   &MoveManager::encoder_monitor, 
                                   this);

    ROS_INFO("initialized the Move Manager");

}


/**
 * NOTE: This is a dangerous design because there is a scenario where multiple
 *       threads are attempting to read/write to the same point in memory thus
 *       causing the process to fail. 
 * 
 * TODO: Add a mutex or something to lock the shared memory
 */
void MoveManager::encoder_monitor (const daedalus_msgs::teensy_message::ConstPtr& msg)
{
    for (int ii = 0; ii < NUM_JOINTS; ++ii)
    {
        current_enc_pos[ii] = msg->steps[ii];
    }
}


/**
 * Is the movement complete? Checks that the current encoder positions are 
 * withtin the specified tolerance.
 *  
 * @return - true, movement is complete
 */
bool MoveManager::is_it_complete (void)
{
    std::vector<bool> complete;
    bool retval = false;

    for (int ii = 0; ii < NUM_JOINTS; ii++)
    {
        if (fabs(current_enc_pos[ii] - desired_enc_pos[ii]) <= move_tolerance[ii])
        {
            retval = true;
        }
        else 
        {
            retval = false;
        }
    }

    if (retval)
    {
        for (int ii = 0; ii < NUM_JOINTS; ii++)
        {
            ROS_INFO("Complete: %d", ii);
        }
    }

    return retval;
}


/**
 * Joint Pose Command Service
 * 
 * Is called by the mission state machine to send the arm to a pre-determined 
 * pose.
 * 
 * @return - res.done indicates the completion of the service. ROS standards 
 *           dictate that true should be returned once the function should 
 *           be exited.
 */
bool MoveManager::joint_pose_cmd (daedalus_msgs::joint_pose_cmd::Request &req,
                                  daedalus_msgs::joint_pose_cmd::Response &res)
{
    daedalus_msgs::teensy_message tnsy_msg;
    std::string param;
    bool timeout = false;
    bool complete = false;
    ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time;
    std::vector<double> joint_group_positions;
    ros::Rate rate(10);

    param =  "/daedalus/" + req.pose_name;

    ROS_INFO("Moving to joint state: %s", param.c_str());
    
    if (ros::param::get(param, joint_group_positions))
    {

        // publish the message
        for (int ii = 0; ii < NUM_JOINTS; ++ii) 
        { 
            tnsy_msg.steps.push_back(joint_group_positions[ii]);
            desired_enc_pos[ii] = joint_group_positions[ii];
        }

        joint_position_cmd.publish(tnsy_msg);

        // monitor for completion
        start_time = ros::Time::now();
        while (ros::ok())
        {
            complete = is_it_complete();

            if (complete)
            {
                res.done = true;
                return true;
            }

            elapsed_time = ros::Time::now() - start_time;
            if (elapsed_time.toSec() > move_timeout)
            {
                ROS_WARN("Movement timed out.");
                res.done = false;
                return true;
            }

            rate.sleep();
        }
    } 
    else 
    {
        ROS_WARN("Joint pose does not exist!");
        res.done = false;
        return true;
    }

    res.done = false;
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
