#include "gimbal_serializer.h"

namespace gimbal_serializer{


GimbalSerializer::GimbalSerializer():
    nh_(ros::NodeHandle())
{

}

} //end gimbal_serializer namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gimbal_serial_node");
    gimbal_serializer::GimbalSerializer gimbal_serial_node;
    ros::spin();
    return 0;
}
