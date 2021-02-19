#ifndef _KINOVADRV_H
#define _KINOVADRV_H

// C library headers
#include <stdio.h>
#include <string.h>

// C++ library headers
#include <string>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <mutex>

#define MAX_READ_RETRY 10
#define MAX_WRITE_RETRY 10
namespace KinovaApi
{
    class CommLayer
    {
    public:
        //This structure represents a RS-485 message
        typedef struct message_s
        {
            //Command ID of the message. Use #define from the COMMAND ID LIST above.
            short Command;

            //Source of the message. If this is an actuator, it will be an address.
            unsigned char SourceAddress;

            //Destination of the message. Use the address of the actuator.
            unsigned char DestinationAddress;

            //Data of the message displayed as unsigned char, float or unsigned long.
            union
            {
                unsigned char DataByte[16];
                uint16_t DataShort[8];
                float DataFloat[4];
                unsigned int DataLong[4];
            };
        } message_t;

        typedef enum CommStatus_s
        {
            COMM_OK = 0,
            COMM_INIT_ERROR,
            COMM_WRITE_ERROR,
            COMM_READ_ERROR
        } CommStatus_t;

        CommLayer();
        CommStatus_t commLayerClose();
        CommStatus_t commLayerInit(std::string device, speed_t baudrate);
        CommStatus_t commLayerRead(message_t *message_buffer, const int nb_msg_to_read, int &nb_msg_read);
        CommStatus_t commLayerWrite(message_t *message_buffer, const int nb_msg_to_send, int &nb_msg_sent);

    private:
        std::string device_;
        unsigned int baudrate_;
        bool init_flag_;
        struct termios tty_settings_;
        int serial_port_;
    };

    class APILayer
    {
    public:
        typedef enum ApiStatus_s
        {
            API_OK = 0,
            API_INIT_ERROR,
            API_WRITE_ERROR,
            API_READ_ERROR

        } ApiStatus_t;

    private:
        typedef struct kinova_actuator_s
        {
            uint32_t address;
            uint32_t posHalls;
            uint32_t posOpticals;
            uint32_t speed;
            uint32_t torque;
            uint32_t current;
            uint32_t pwm;
            uint32_t command;
            uint16_t temperature;
            uint32_t sensor_temperature;
            uint32_t max_temperature;
            uint16_t accelX;
            uint16_t accelY;
            uint16_t accelZ;
            uint32_t fw_version;
            uint32_t id_type;
            uint32_t error;
            /* Encoder status */
            uint32_t is_encoder_ctrl_activated;
            uint32_t is_encoder_index_valid;
            uint32_t is_encoder_indexed;
        } kinova_actuator_t;

    public:
        APILayer();
        virtual ~APILayer(){};
        ApiStatus_t init(const std::string device, const bool &debug_log = false);
        ApiStatus_t deviceInitialisation(const uint16_t &jointAddress, float &jointPosition);
        ApiStatus_t startMotorControl(const uint16_t &jointAddress);
        ApiStatus_t stopMotorControl(const uint16_t &jointAddress);
        ApiStatus_t getActualPosition(const uint16_t &jointAddress, float &jointCurrent, float &jointPositionHall, float &jointSpeed, float &jointTorque);
        ApiStatus_t setCommandAllValue(const uint16_t &jointAddress, const float &jointCommand, float &jointCurrent, float &jointPositionHall,
                                       float &jointSpeed, float &jointTorque, float &jointPMW, float &jointPositionOptical,
                                       short &jointAccelX, short &jointAccelY, short &jointAccelZ, short &jointTemp);
        ApiStatus_t setPositionCommand(const uint16_t& jointAddress, const float& jointCommand, float& jointCurrent,
                                 float& jointPositionHall, float& jointSpeed, float& jointTorque);
        ApiStatus_t getPosition();

    private:
        CommLayer comm_;
        const int getExpectedReply_(uint16_t command);
        ApiStatus_t readWrite_(CommLayer::message_t *writeMessage, CommLayer::message_t *readMessage, const int &expectedResponseMsg);
        std::mutex api_mutex_;
    };

} // namespace KinovaApi

// the class of the plugin factory
#include <iostream>

class ApiFactory //: factory
{
public:
    KinovaApi::APILayer *makedyn()
    {
        std::cout << "    making foo [foo.cc ApiFactory::makedyn()]" << std::endl;
        return new KinovaApi::APILayer;
    }
};

#endif