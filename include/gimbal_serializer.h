#ifndef GIMBAL_SERIALIZER_H
#define GIMBAL_SERIALIZER_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

#include "serial_exception.h"

#define GIMBAL_SERIAL_OUT_START_BYTE 0xA5
#define GIMBAL_SERIAL_OUT_PAYLOAD_LENGTH 8
#define GIMBAL_SERIAL_OUT_MSG_LENGTH 10

#define GIMBAL_SERIAL_IN_START_BYTE 0xA5
#define GIMBAL_SERIAL_IN_PAYLOAD_LENGTH 8
#define GIMBAL_SERIAL_IN_MSG_LENGTH 10

#define GIMBAL_SERIAL_READ_BUF_SIZE 256

namespace gimbal_serializer{

class GimbalSerial
{
public:
    GimbalSerial(std::string port,
                 int baud_rate,
                 const boost::function<void (geometry_msgs::Vector3Stamped::ConstPtr)> &callback);
    ~GimbalSerial();
    void sendCommand(geometry_msgs::Vector3Stamped::ConstPtr msg);
    void close();

private:
    //===========================================================================
    // definitions
    //===========================================================================

    /**
       * \brief Output message buffer to be put on the write queue
       */
    struct WriteBuffer
    {
        uint8_t data[GIMBAL_SERIAL_OUT_MSG_LENGTH]; //!< data buffer
        size_t pos; //!< index of next byte to be transmitted

        /**
         * \brief Default constructor
         */
        WriteBuffer() : pos(0) {}

        /**
         * \brief Get address of next byte to be transmitted
         * \return Pointer to next byte to be transmitted
         */
        uint8_t * dpos()
        {
            return data + pos;
        }

        /**
         * \brief Compute how many bytes are left to be transmitted
         * \return The number of bytes remaining to be transmitted
         */
        size_t nbytes()
        {
            return GIMBAL_SERIAL_OUT_MSG_LENGTH - pos;
        }
    };

    /**
       * \brief States for the incoming message parsing state machine
       */
    enum ParseState
    {
        PARSE_STATE_IDLE, //!< waiting for the start byte
        PARSE_STATE_GOT_START_BYTE, //!< got the start byte; processing the payload
        PARSE_STATE_GOT_PAYLOAD //!< got the payload; check CRC
    };

    /**
       * \brief Convenience typedef for mutex lock
       */
    typedef boost::lock_guard<boost::recursive_mutex> mutex_lock;

    //===========================================================================
    // methods
    //===========================================================================

    /**
       * \brief Initiate an asynchronous read operation
       */
    void do_async_read();

    /**
       * \brief Handler for end of asynchronous read operation
       * \param error Error code
       * \param bytes_transferred Number of bytes received
       */
    void async_read_end(const boost::system::error_code& error, size_t bytes_transferred);

    /**
       * \brief Initiate an asynchrounous write operation
       */
    void do_async_write(bool check_write_state);

    /**
       * \brief Handler for the end of an asynchronous write operation
       * \param error Error code
       * \param bytes_transferred Number of bytes transmitted
       */
    void async_write_end(const boost::system::error_code& error, std::size_t bytes_transferred);

    /**
       * \brief Parse the next byte in an incoming message
       * \param c The byte to be parsed
       * \return True if a complete message has been received and parsed successfully
       *
       * This function implements the state machine for parsing incoming messages.
       */
    bool parse_byte(uint8_t c);

    /**
       * \brief Unpack the contents of the payload buffer into a geometry_msgs/vector3stamped message
       * \param[out] msg The message for which to populate the data
       */
    void unpack_payload(geometry_msgs::Vector3Stamped::Ptr msg);

    /**
       * \brief Pack the contents of a geometry_msgs/Vector3stamped message into the TX buffer
       * \param[out] buf The buffer of raw bytes to be transmitted
       * \param[in] msg The message whose contents are to be buffered
       * \return The number of bytes written to the buffer
       */
    size_t pack_out_msg(uint8_t buf[], geometry_msgs::Vector3Stamped::ConstPtr msg);

    /**
       * \brief Update the value of the CRC-8-CCITT checksum with the next byte
       * \param crc The current value of the checksum
       * \param data The byte to be processed
       * \return The updated value of the checksum
       *
       * source: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#gab27eaaef6d7fd096bd7d57bf3f9ba083
       */
    uint8_t crc8_ccitt_update(uint8_t crc, uint8_t data);

    //===========================================================================
    // member variables
    //===========================================================================

    boost::asio::io_service io_service_; //!< boost io service provider
    boost::asio::serial_port serial_port_; //!< boost serial port object
    boost::thread io_thread_; //!< thread on which the io service runs
    boost::recursive_mutex mutex_; //!< mutex for threadsafe operation

    boost::function<void (geometry_msgs::Vector3Stamped::ConstPtr)> data_callback_; //!< pointer to function to call when new data is ready

    uint8_t read_buf_raw_[GIMBAL_SERIAL_READ_BUF_SIZE]; //!< buffer for raw bytes received

    std::list<WriteBuffer*> write_queue_; //!< queue of buffers to be written to the serial port
    bool write_in_progress_; //!< flag for whether async_write is already running

    ParseState parse_state_; //!< the current state for the incoming message parsing state machine
    uint8_t payload_buf_raw_[GIMBAL_SERIAL_IN_PAYLOAD_LENGTH]; //!< buffer for the payload of an incoming message
    int payload_index_; //!< current index in the payload buffer for an incoming message
    uint8_t crc_value_; //!< current value of the CRC checksum for an incoming message


};

} //end namespace
#endif // GIMBAL_SERIALIZER_H
