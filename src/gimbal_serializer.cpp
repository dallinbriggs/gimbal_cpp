#include "gimbal_serializer.h"

namespace gimbal_serializer{

using boost::asio::serial_port_base;

GimbalSerial::GimbalSerial(std::string port,
                           int baud_rate,
                           const boost::function<void (geometry_msgs::Vector3Stamped::ConstPtr)> &callback):
    io_service_(),
    serial_port_(io_service_),
    data_callback_(callback),
    parse_state_(PARSE_STATE_IDLE),
    write_in_progress_(false)
{
    // setup serial port
    try
    {
        serial_port_.open(port);
        serial_port_.set_option(serial_port_base::baud_rate(baud_rate));
        serial_port_.set_option(serial_port_base::character_size(8));
        serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
        serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    }
    catch (boost::system::system_error e)
    {
        throw SerialException(e);
    }

    // start reading from serial port
    do_async_read();
    io_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &this->io_service_));
}

GimbalSerial::~GimbalSerial()
{
    close();
}

void GimbalSerial::close()
{
  mutex_lock lock(mutex_);

  io_service_.stop();
  serial_port_.close();

  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void GimbalSerial::do_async_read()
{
  if (!serial_port_.is_open()) return;

  serial_port_.async_read_some(
        boost::asio::buffer(read_buf_raw_, GIMBAL_SERIAL_READ_BUF_SIZE),
        boost::bind(
          &GimbalSerial::async_read_end,
          this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}


} // end namespace

int main(int argc, char** argv)
{
    return 0;
}
