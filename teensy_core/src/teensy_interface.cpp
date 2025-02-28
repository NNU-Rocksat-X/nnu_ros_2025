/**
 * Updated and simplified teensy interface
 *
 * @brief updates setpoints for robotic arm positions, returns encoder values
 *
 * @author Riley Mark 
 * @author Febuary 27, 2025
 * 
 ******************************************************************************/

#include <ros/ros.h>
#include <fstream>

#include "daedalus_msgs/teensy_message.h"
#include "teensy_comm.h"

#define TEENSY_SER_PORT "/dev/ttyACM0"
#define TEENSY_BAUD_RATE 115200
#define LOOP_RATE 50

teensy_command_t tnsy_cmd = {0};
teensy_status_t tnsy_sts = {0};

uint16_t gear_ratio[NUM_JOINTS] = {1, 2, 3, 4, 5, 6};


void position_callback(const daedalus_msgs::teensy_message::ConstPtr& msg)
{
    double deg_sec = 0.0;
    double deg_ms = 0.0;
    int32_t step_ms = 0;

    for (int ii = 0; ii < NUM_JOINTS; ii++)
    {
        tnsy_cmd.setpoint_position[ii] = msg->steps[ii];
    }

    ROS_INFO("Joint positions updated.");
}

/**
 * M A I N
 * 
 */
int main (int argc, char** argv)
{
    ros::init(argc, argv, "teensy_interface");
    ros::NodeHandle nh;
    ROS_INFO("Starting Teensy Interface");

    ros::Subscriber sub_0 = nh.subscribe("joint_position_cmd",
                                         2, 
                                         &position_callback);
    ROS_INFO("Created joint_position_cmd subscriber");

    ros::Rate loop_rate(LOOP_RATE);
    while(ros::ok())
    {
        // TODO: read from teensy file descriptor 
        // TODO: write to teensy file descriptor 

        loop_rate.sleep();
    }

}