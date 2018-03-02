#ifndef GIMBAL_SERIALIZER_H
#define GIMBAL_SERIALIZER_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <functional>

#include "async_comm/serial.h"

#define NUM_BYTES 8

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

    // Params
    std::string port_;
    int baudrate_;

    // Callbacks and functions
    void command_callback(const geometry_msgs::Vector3StampedConstPtr &msg);
    void serialize_msg(float x, float y, float z);
    void init_serial();
    void serial_receive(uint8_t byte);

    // Serialization
    async_comm::Serial *serial_;

};

#endif // GIMBAL_SERIALIZER_H
} //end namespace gimbal_serializer

