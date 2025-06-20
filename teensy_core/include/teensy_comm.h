/**
 * @file teensy_comm.h
 *
 * @brief message structures for the teensy command and status messages.
 *
 * @author Riley Mark
 * @author Febuary 27, 2025
 *
 ******************************************************************************/

#ifndef TEENSY_COMM_H
#define TEENSY_COMM_H

#include <stdint.h>
#include <cstddef>

#define NUM_JOINTS 8
#define DEG_PER_ENC_STEP 1.8 // TODO: someone update this
#define ENC_STEP_PER_DEG 0.555555556 // TODO: same with this one
#define CRC_POLY 1021
#define CRC_INIT 0xFFFF 


/**
 * This is packed so that there is no padding around the byte boundaries. This
 * ensures reliable parsing on both sides of the serial communication pipe.
 */  
#pragma pack(1)

typedef struct _teensy_header_t
{
    uint16_t header;
    uint16_t seq;
    uint16_t len;
    uint16_t type;
} teensy_header_t;

typedef struct _teensy_command_t
{
    teensy_header_t hdr;

    int16_t setpoint_position[NUM_JOINTS];
    uint16_t led_state;

    uint16_t crc;
} teensy_command_t;

typedef struct _teensy_status_t
{
    teensy_header_t hdr;

    int16_t encoder[NUM_JOINTS];
    uint16_t debug_feild_0;
    uint16_t debug_feild_1;

    uint16_t crc;
} teensy_status_t;

#pragma pack()

/**
 * Converts encoder steps to radians
 * 
 * @param enc_steps - raw encoder values
 * @param gear_ratio - the gear ratio of the joint
 * 
 * @return - angle in radians
 */
double enc_steps_to_rad (int32_t enc_steps, uint16_t gear_ratio);

/**
 * Converts encoder steps to radians
 * 
 * @param rad - angle in radians
 * @param gear_ratio - the gear ratio of the joint
 * 
 * @return - angle converted to encoder steps
 */
int32_t rad_to_enc_steps (double rad, uint16_t gear_ratio);

/**
 * CRC16 -- Google this. It's one mechanism that allows data to be transferred
 * between devices and across networks without loosing integrity.
 * 
 * @param data - character buffer of message being sent
 * @param length - length of the data buffer
 * 
 * @return - unsigned 16 bit crc calculated on the entire payload (minus crc obv)
 */
uint16_t crc16_ccitt (const uint8_t *data, size_t length);

#endif
