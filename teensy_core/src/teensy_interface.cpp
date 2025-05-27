/**
 * Updated and simplified teensy interface
 *
 * @brief updates setpoints for robotic arm positions, returns encoder values
 *
 * @author Riley Mark 
 * @author Febuary 27, 2025
 * 
 * TODO: Add service for updating arm position
 * TODO: Gear ratios and step to degree values?
 * 
 ******************************************************************************/

#include <ros/ros.h>
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
#define TEENSY_SER_PORT "/dev/ttyTHS1"
#define TEENSY_BAUD_RATE 115200
#define LOOP_RATE 50
#define PRINT_DATA 1 // 0 - no print, 1 - print data

static teensy_command_t tnsy_cmd = {0};
static teensy_status_t tnsy_sts = {0};

uint16_t gear_ratio[NUM_JOINTS] = {1, 2, 3, 4, 5, 6, 7, 8};

/**
 * Position callback - This callback function updates teensy command position
 *                     setpoints. The Joint pose command service calls this.
 */
void position_callback(const daedalus_msgs::teensy_message::ConstPtr& msg)
{
    ROS_WARN("position Callback");
    double deg_sec = 0.0;
    double deg_ms = 0.0;
    int32_t step_ms = 0;

    for (int ii = 0; ii < NUM_JOINTS; ii++)
    {
        tnsy_cmd.setpoint_position[ii] = (uint16_t)msg->steps[ii];
    }

    // this isnt a great place for this.. Generally callback should be kept 
    // lean, but who is keeping track amirite
    tnsy_cmd.hdr.header = 0x5555;
    tnsy_cmd.hdr.seq++;
    tnsy_cmd.hdr.len = sizeof(tnsy_cmd);

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
        ROS_WARN("did not pass header");
        return 1;
    }
}


/**
 * Initializes the serial port. 
 * 
 * Settings:
 *  - baud 115200
 *  - 8 data 1 stop bit (standard)
 *  - ignore modem control lines
 *  - read once 26 bytes are available (not standard but this will help debugging)
 *  - timeout of 1 second
 * 
 * @param port - the serial port that the teensy is connected to (ex. /dev/ttyACM1)
 * 
 * @return - file descriptor
 */
int init_serial (const char* port) 
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) 
    {
        perror("Error opening serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    cfmakeraw(&options);  // Set raw mode

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver, ignore modem control lines

    options.c_cc[VMIN]  = sizeof(tnsy_sts);  // Read only when 26 bytes available, tune this param
    options.c_cc[VTIME] = 10;  // Timeout = 1 second

    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIFLUSH);

    return fd;
}


void generate_command_message ()
{
    tnsy_cmd.hdr.header = 0x5555;
    tnsy_cmd.hdr.seq++;
    tnsy_cmd.hdr.len = sizeof(tnsy_cmd);
    tnsy_cmd.hdr.type = 1;
}

void print_info (void)
{
    ROS_INFO("------------ Teensy Status ------------");
    ROS_INFO("%04x %d %d %d",
             tnsy_sts.hdr.header, 
             tnsy_sts.hdr.seq, 
             tnsy_sts.hdr.len, 
             tnsy_sts.hdr.type);
    ROS_INFO("%d %d %d %d %d %d %d %d",
             tnsy_sts.encoder[0],
             tnsy_sts.encoder[1],
             tnsy_sts.encoder[2],
             tnsy_sts.encoder[3],
             tnsy_sts.encoder[4],
             tnsy_sts.encoder[5],
             tnsy_sts.encoder[6],
             tnsy_sts.encoder[7]);
    ROS_INFO("%d %d %d",
             tnsy_sts.debug_feild_0,
             tnsy_sts.debug_feild_1,
             tnsy_sts.crc);

    ROS_INFO("------------ Teensy Command ------------");
    ROS_INFO("%04x %d %d %d",
            tnsy_cmd.hdr.header, 
            tnsy_cmd.hdr.seq, 
            tnsy_cmd.hdr.len, 
            tnsy_cmd.hdr.type);
    ROS_INFO("%d %d %d %d %d %d %d %d",
            tnsy_cmd.setpoint_position[0],
            tnsy_cmd.setpoint_position[1],
            tnsy_cmd.setpoint_position[2],
            tnsy_cmd.setpoint_position[3],
            tnsy_cmd.setpoint_position[4],
            tnsy_cmd.setpoint_position[5],
            tnsy_cmd.setpoint_position[6],
            tnsy_cmd.setpoint_position[7]);
    ROS_INFO("%d %d",
            tnsy_cmd.led_state,
            tnsy_cmd.crc);
    ROS_INFO("\n");
}

/**
 * M A I N
 * 
 */
int main (int argc, char** argv)
{
    int ser_fd = 0;
    uint8_t in_buf[26]; // TODO: Is this valid?
    uint8_t out_buf[26];
    int bytes = -1;

    daedalus_msgs::teensy_message encdr_vals;

    ros::init(argc, argv, "teensy_interface");
    ros::NodeHandle nh;
    ROS_INFO("Starting Teensy Interface");

    ros::Subscriber sub_0 = nh.subscribe("update_teensy_cmd",
                                         500, 
                                         &position_callback);
    ROS_INFO("Created update_teensy_cmd subscriber");

    ros::Publisher pub_0 = nh.advertise<daedalus_msgs::teensy_message>("encoder_values", 
                                                                       10);
    ROS_INFO("encoder_values publisher is created");

    ser_fd = init_serial(TEENSY_SER_PORT);

    ros::Rate loop_rate(LOOP_RATE);
    while(ros::ok())
    {
        bytes = read(ser_fd, &in_buf, sizeof(tnsy_sts));
        // ROS_INFO("read in %d bytes as %d", bytes, in_buf[0]);
        // ROS_INFO("print");

        if (bytes >= 0)
        {
            // ROS_INFO("read in %d bytes", bytes);
            if (parse_message(in_buf, bytes) == 0)
            {
                print_info();

                for (int ii = 0; ii < NUM_JOINTS; ++ii)
                {
                    encdr_vals.steps.push_back((double)tnsy_sts.encoder[ii]);
                }

                pub_0.publish(encdr_vals);
                encdr_vals.steps.clear();

            }
        }

        generate_command_message();

        memcpy(out_buf, &tnsy_cmd, sizeof(tnsy_cmd) - 2);
        tnsy_cmd.crc = crc16_ccitt(out_buf, sizeof(tnsy_cmd) - 2); // crc is the 2 bytes

        memcpy(out_buf, &tnsy_cmd, sizeof(tnsy_cmd));
        write(ser_fd, out_buf, sizeof(tnsy_cmd));
        //ROS_INFO("serial command written");

        ros::spinOnce();
        loop_rate.sleep();

    }
}
