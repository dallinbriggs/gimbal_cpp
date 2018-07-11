#include "gimbal_serializer.h"

namespace gimbal_serializer{


GimbalSerializer::GimbalSerializer():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
{
    // Set/get params
    nh_private_.param<std::string>("port", port_, "/dev/gimbal");
    nh_private_.param<int>("baudrate", baudrate_, 115200);
    nh_private_.param<int>("rc_channel", rc_channel_, 7);
    nh_private_.param<float>("retract_up_angle", retract_up_angle_, 0);
    nh_private_.param<float>("retract_down_angle", retract_down_angle_, 1.57);

    // Initialize serial stuff
    init_serial();

    // Setup ros subscribers and publishers
    command_sub = nh_.subscribe("gimbal/control", 1, &GimbalSerializer::command_callback, this);
    retract_sub = nh_.subscribe("mavros/rc/in", 1, &GimbalSerializer::retract_callback, this);
    command_echo_pub = nh_.advertise<gimbal_serializer::status>("gimbal/status", 1);
    parse_state = PARSE_STATE_IDLE;
    crc_error_count = 0;
}

void GimbalSerializer::command_callback(const geometry_msgs::Vector3StampedConstPtr &msg)
{
    x_command = msg->vector.x;
    y_command = msg->vector.y;
    z_command = msg->vector.z;
    retract_command = retract_down_angle_;
    if (retract_rc_in > 1500)
    {
        x_command = 0;
        y_command = 0;
        z_command = 0;
        retract_command = retract_down_angle_;
        sleep(1);
        serialize_msg();
        retract_command = retract_up_angle_;
        serialize_msg();
    }
    serialize_msg();
}

void GimbalSerializer::retract_callback(const mavros_msgs::RCInConstPtr &msg)
{
    retract_rc_in = msg->channels[rc_channel_];
}

void GimbalSerializer::serialize_msg()
{
    uint8_t buf[SERIAL_OUT_MSG_LENGTH];
    buf[0] = SERIAL_OUT_START_BYTE;

    memcpy(buf+1, &x_command, sizeof(float));
    memcpy(buf+5, &y_command, sizeof(float));
    memcpy(buf+9, &z_command, sizeof(float));
    memcpy(buf+13, &retract_command, sizeof(float));

    uint8_t crc_value = SERIAL_CRC_INITIAL_VALUE;
    for (int i = 0; i < SERIAL_OUT_MSG_LENGTH - SERIAL_CRC_LENGTH; i++)
    {
        crc_value = out_crc8_ccitt_update(crc_value, buf[i]);
    }
    buf[SERIAL_OUT_MSG_LENGTH - 1] = crc_value;
    serial_->send_bytes(buf, SERIAL_OUT_MSG_LENGTH);
}

void GimbalSerializer::init_serial()
{
    serial_ = new async_comm::Serial(port_, (unsigned int)baudrate_);
    serial_->register_receive_callback(std::bind(&GimbalSerializer::rx_callback, this, std::placeholders::_1));

    if (!serial_->init())
    {
        std::printf("Failed to initialize serial port\n");
    }
}

uint8_t GimbalSerializer::out_crc8_ccitt_update(uint8_t outCrc, uint8_t outData)
{
    uint8_t   i;
    uint8_t   data;

    data = outCrc ^ outData;

    for ( i = 0; i < SERIAL_OUT_PAYLOAD_LENGTH; i++ )
    {
        if (( data & 0x80 ) != 0 )
        {
            data <<= 1;
            data ^= 0x07;
        }
        else
        {
            data <<= 1;
        }
    }
    return data;

}

uint8_t GimbalSerializer::in_crc8_ccitt_update(uint8_t inCrc, uint8_t inData)
{
    uint8_t   i;
    uint8_t   data;

    data = inCrc ^ inData;

    for ( i = 0; i < SERIAL_IN_PAYLOAD_LENGTH; i++ )
    {
        if (( data & 0x80 ) != 0 )
        {
            data <<= 1;
            data ^= 0x07;
        }
        else
        {
            data <<= 1;
        }
    }
    return data;

}

void GimbalSerializer::unpack_in_payload(uint8_t buf[], float *command_frequency, float *servo_frequency, float *roll, float *pitch, float *yaw, float *retract)
{
    memcpy(command_frequency, buf, 4);
    memcpy(servo_frequency, buf + 4, 4);
    memcpy(roll, buf + 8, 4);
    memcpy(pitch, buf + 12, 4);
    memcpy(yaw, buf + 16, 4);
    memcpy(retract, buf + 20, 4);
}

void GimbalSerializer::rx_callback(uint8_t byte)
{
    if (parse_in_byte(byte))
    {
        float roll, pitch, yaw, retract;
        float command_hz;
        float servo_hz;
        unpack_in_payload(in_payload_buf, &command_hz, &servo_hz, &roll, &pitch, &yaw, &retract);
        gimbal_serializer::status msg;
        msg.command_in_Hz = command_hz;
        msg.servo_command_Hz = servo_hz;
        msg.roll_command = roll;
        msg.pitch_command = pitch;
        msg.yaw_command = yaw;
        msg.header.stamp = ros::Time::now();
        command_echo_pub.publish(msg);
    }
}

//==================================================================
// handle an incoming byte
//==================================================================
bool GimbalSerializer::parse_in_byte(uint8_t c)
{
    bool got_message = false;
    switch (parse_state)
    {
    case PARSE_STATE_IDLE:
        if (c == SERIAL_IN_START_BYTE)
        {
            in_crc_value = SERIAL_CRC_INITIAL_VALUE;
            in_crc_value = in_crc8_ccitt_update(in_crc_value, c);

            in_payload_index = 0;
            parse_state = PARSE_STATE_GOT_START_BYTE;
        }
        break;

    case PARSE_STATE_GOT_START_BYTE:
        in_crc_value = in_crc8_ccitt_update(in_crc_value, c);
        in_payload_buf[in_payload_index++] = c;
        if (in_payload_index == SERIAL_IN_PAYLOAD_LENGTH)
        {
            parse_state = PARSE_STATE_GOT_PAYLOAD;
        }
        break;

    case PARSE_STATE_GOT_PAYLOAD:
        if (c == in_crc_value)
        {
            got_message = true;
        }
        else
        {
            crc_error_count += 1;
        }
        parse_state = PARSE_STATE_IDLE;
        break;
    }

    return got_message;
}

} //end gimbal_serializer namespace


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gimbal_serial_node");
    gimbal_serializer::GimbalSerializer gimbal_serial_node;
    ros::spin();
    return 0;
}
