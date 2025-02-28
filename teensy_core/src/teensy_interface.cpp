/**
 * Updated and simplified teensy interface
 *
 * @brief updates setpoints for robotic arm positions, returns encoder values
 *
 * @author Riley Mark 
 * @author Febuary 27, 2025
 */

#include <ros/ros.h>
#include <fstream>

#include "daedalus_msgs/teensy_message.h"

#define TEENSY_SER_PORT "/dev/ttyACM0"
#define TEENSY_BAUD_RATE 115200
#define LOOP_RATE 50
#define NUM_JOINTS 6

void positionCB(const daedalus_msgs::teensy_message::ConstPtr& msg)
{
    double deg_sec = 0.0;
    double deg_ms = 0.0;
    int32_t step_ms = 0;

    for (int ii = 0; ii < NUM_JOINTS; ii++)
    {
        deg_ms = msg->steps[ii];
        step_ms = (int32_t)(deg_ms * enc_steps_per_rad[ii]);
        tx.joint_velocity_cmd[ii] = step_ms;
    }

    ROS_INFO("Joint positions updated.");
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "teensy_interface");
    ros::NodeHandle nh;

    ros::Subscriber sub_0 = nh.subscribe("joint_position_cmd", 2, &positionCB);

    ros::Rate loop_rate(LOOP_RATE);
    while(ros::ok())
    {
        // do something
        loop_rate.sleep();
    }

}