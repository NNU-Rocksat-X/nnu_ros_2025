/**
 * Updated and simplified teensy interface
 *
 * @brief updates setpoints for robotic arm positions, returns encoder values
 *
 * @author Riley Mark 
 * @author Febuary 27, 2025
 * 
 * TODO: test the serial communications
 * 
 ******************************************************************************/

#include <ros/ros.h>
#include <stdlib.h>
#include <stdlib.h> 
#include <fstream>
#include <fcntl.h>      
#include <termios.h>    
#include <unistd.h>     
#include <errno.h>      
#include <cstring>   

#include "daedalus_msgs/teensy_message.h"
#include "teensy_comm.h"

// TODO: if you want to use the pins to communicate, you have to turn off the 
// serial console and you have to turn on UART0
#define TEENSY_SER_PORT "/dev/ttyS0" // TODO: change this to THS1 or whatever
#define TEENSY_BAUD_RATE 115200
#define LOOP_RATE 50

static teensy_command_t tnsy_cmd = {0};
static teensy_status_t tnsy_sts = {0};

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
 * Parses teensy messages 
 * 
 * @return - 0 success, 1 invalid header, 2 invalid crc
 */
int parse_message (const uint8_t* buf, int size) 
{
    uint16_t calc_crc = 0;
    uint16_t rec_crc = 0;
    uint16_t hdr_chk = 0;

    // crc16

    memcpy(&hdr_chk, buf, sizeof(hdr_chk));
    if (hdr_chk == 0x5555)
    {
        ROS_INFO("beginning of message found");

        calc_crc = crc16_ccitt(buf, size - 2);
        memcpy(&rec_crc, buf + size - 2, sizeof(rec_crc));
        if (calc_crc == rec_crc)
        {
            ROS_INFO("Passed CRC.");
            memcpy(&tnsy_sts, buf, size - 2);

            return 0;
        }
        else
        {
            return 2;
        }
    }
    else
    {
        return 1;
    }

    return 0;
}

// TOOD: add message builder function to teensy comm
// TODO: create daedalus core 
// TODO: Come up with a way that the folks at NNU can test it


/**
 * M A I N
 * 
 */
int main (int argc, char** argv)
{
    int ser_fd = 0;
    uint8_t* in_buf = (uint8_t*)malloc(1024); // TODO: Is this valid?
    uint8_t* out_buf = (uint8_t*)malloc(1024);
    int bytes = 0;

    ros::init(argc, argv, "teensy_interface");
    ros::NodeHandle nh;
    ROS_INFO("Starting Teensy Interface");

    ros::Subscriber sub_0 = nh.subscribe("joint_position_cmd",
                                         2, 
                                         &position_callback);
    ROS_INFO("Created joint_position_cmd subscriber");

    ser_fd = open(TEENSY_SER_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (ser_fd == -1) 
    {
        ROS_ERROR("Teensy File descriptor is invalid.");
    }

    struct termios tty;
    if (tcgetattr(ser_fd, &tty) != 0) {
        ROS_ERROR("Error getting terminal attributes.");
        close(ser_fd);
        return -1;
    }

    // Set baud rate to 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Configure 8N1 (8 data bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;

    // Disable flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // Apply settings
    if (tcsetattr(ser_fd, TCSANOW, &tty) != 0) {
        ROS_ERROR("Error setting the terminal attributes");
        close(ser_fd);
        return -1;
    }

    ros::Rate loop_rate(LOOP_RATE);
    while(ros::ok())
    {

        bytes = read(ser_fd, in_buf, sizeof(teensy_status_t));

        if (bytes > 0)
        {
            parse_message(in_buf, bytes);
        }

        memcpy(out_buf, &tnsy_cmd, sizeof(tnsy_cmd));
        write(ser_fd, out_buf, sizeof(tnsy_cmd));

        loop_rate.sleep();
    }

}