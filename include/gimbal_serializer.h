#ifndef GIMBAL_SERIALIZER_H
#define GIMBAL_SERIALIZER_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

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
    geometry_msgs::Vector3Stamped gimbal_command;
    ros::Subscriber command_sub;
    void command_callback(const geometry_msgs::Vector3StampedConstPtr &msg);
};

#endif // GIMBAL_SERIALIZER_H
} //end namespace gimbal_serializer

