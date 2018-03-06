#ifndef GIMBAL_SERIALIZER_H
#define GIMBAL_SERIALIZER_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <functional>

#include "async_comm/serial.h"

#define SERIAL_CRC_LENGTH 1
#define SERIAL_CRC_INITIAL_VALUE 0x00

#define SERIAL_OUT_START_BYTE 0xA5
#define SERIAL_OUT_PAYLOAD_LENGTH 12
#define SERIAL_OUT_MSG_LENGTH 14

namespace gimbal_serializer
{

class GimbalSerializer
{
public:
    GimbalSerializer();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    geometry_msgs::Vector3Stamped gimbal_command;
    ros::Subscriber command_sub;
    ros::Publisher command_echo;

    //Variables
    float x_command;
    float y_command;
    float z_command;
    uint8_t in_crc_value;

    // Params
    std::string port_;
    int baudrate_;

    // Callbacks and functions
    void command_callback(const geometry_msgs::Vector3StampedConstPtr &msg);
    void serialize_msg();
    void init_serial();
    void serial_receive(uint8_t byte);
    uint8_t crc8_ccit_update(uint8_t inCrc, uint8_t inData);

    // Serialization
    async_comm::Serial *serial_;

};

#endif // GIMBAL_SERIALIZER_H
} //end namespace gimbal_serializer

