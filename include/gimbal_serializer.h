#ifndef GIMBAL_SERIALIZER_H
#define GIMBAL_SERIALIZER_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <functional>
#include <mavros_msgs/RCIn.h>
#include <unistd.h>

#include "async_comm/serial.h"
#include "gimbal_serializer/status.h"

#define SERIAL_CRC_LENGTH 1
#define SERIAL_CRC_INITIAL_VALUE 0x00

#define SERIAL_OUT_START_BYTE 0xA5
#define SERIAL_OUT_PAYLOAD_LENGTH 16
#define SERIAL_OUT_MSG_LENGTH 18

#define SERIAL_IN_START_BYTE 0xA5
#define SERIAL_IN_PAYLOAD_LENGTH 24
#define SERIAL_IN_MSG_LENGTH 26


namespace gimbal_serializer
{

class GimbalSerializer
{
public:
    GimbalSerializer();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

//    geometry_msgs::Vector3Stamped gimbal_command;
    ros::Subscriber command_sub;
    ros::Publisher command_echo_pub;
    ros::Subscriber retract_sub;

    //Variables
    float x_command;
    float y_command;
    float z_command;
    float retract_command;
    int retract_rc_in;
    uint8_t in_crc_value;
    uint8_t out_crc_value;

    int in_payload_index;
    uint8_t in_payload_buf[SERIAL_IN_PAYLOAD_LENGTH];
    int crc_error_count;

    // Params
    std::string port_;
    int baudrate_;
    int rc_channel_;
    float retract_up_angle_;
    float retract_down_angle_;

    enum ParseState {
        PARSE_STATE_IDLE,
        PARSE_STATE_GOT_START_BYTE,
        PARSE_STATE_GOT_PAYLOAD
    };

    ParseState parse_state;

    // Callbacks and functions
    void command_callback(const geometry_msgs::Vector3StampedConstPtr &msg);
    void serialize_msg();
    void init_serial();
    void rx_callback(uint8_t byte);
    uint8_t out_crc8_ccitt_update(uint8_t outCrc, uint8_t outData);
    uint8_t in_crc8_ccitt_update(uint8_t inCrc, uint8_t inData);
    bool parse_in_byte(uint8_t c);
    void unpack_in_payload(uint8_t buf[SERIAL_IN_PAYLOAD_LENGTH], float *command_frequency, float *servo_frequency, float *roll, float *pitch, float *yaw, float *retract);
    void retract_callback(const mavros_msgs::RCInConstPtr &msg);

    // Serialization
    async_comm::Serial *serial_;

};

#endif // GIMBAL_SERIALIZER_H
} //end namespace gimbal_serializer

