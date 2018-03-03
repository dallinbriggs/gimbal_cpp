#include "gimbal_serializer.h"

namespace gimbal_serializer{


GimbalSerializer::GimbalSerializer():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
{
    // Set/get params
    nh_private_.param<std::string>("port", port_, "/dev/ttyACM0");
    nh_private_.param<int>("baudrate", baudrate_, 115200);

    // Initialize serial stuff
    init_serial();


    // Setup ros subscribers and publishers
    command_sub = nh_.subscribe("gimbal/command", 1, &GimbalSerializer::command_callback, this);
    command_echo = nh_.advertise<geometry_msgs::Vector3Stamped>("gimbal/current", 1);
}

void GimbalSerializer::command_callback(const geometry_msgs::Vector3StampedConstPtr &msg)
{
    x_command = msg->vector.x;
    y_command = msg->vector.y;
    z_command = msg->vector.z;
    serialize_msg();
}

void GimbalSerializer::serialize_msg()
{
    uint8_t buff[14];
    buff[0] = 0xA5;
    memcpy(buff+1, &x_command, sizeof(float));
    memcpy(buff+5, &y_command, sizeof(float));
    memcpy(buff+9, &z_command, sizeof(float));
    serial_->send_bytes(buff, 14);
}

void GimbalSerializer::init_serial()
{
    serial_ = new async_comm::Serial(port_, (unsigned int)baudrate_);
    serial_->register_receive_callback(std::bind(&GimbalSerializer::serial_receive, this, std::placeholders::_1));

    if (!serial_->init())
    {
        std::printf("Failed to initialize serial port\n");
    }
}

void GimbalSerializer::serial_receive(uint8_t byte)
{
    // This is where you define how to handle a received byte.
}


} //end gimbal_serializer namespace


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gimbal_serial_node");
    gimbal_serializer::GimbalSerializer gimbal_serial_node;
    ros::spin();
    return 0;
}
