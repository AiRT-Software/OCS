
#include <cstring>
#include <cassert>

//#include "../atreyu/include/fcsmodule.h"
#include "aerotoolsGimbalMessageProcessor.h"
#include "stdMessage.h"
#ifdef TESTING
#include "dummySerial.h"
#else
#include "serial.h"
#endif
#include "log.h"
#include "utils.h"

#define MAX_MSG_LENGTH 64

#define SYNC1 0xb5
#define SYNC2 0xca

#define ID_INDEX 2U
#define LENGTH_INDEX 3U
#define PAYLOAD_INDEX 4U

#define MIN_MESSAGE_SIZE 5U

using airt::AeroToolsGimbalMessageProcessor;
using airt::StdMessage;

enum Commands
{
    GOTO_ANGLE = 1,
    SET_SPEED_DEPRECATED,
    MOVE_PITCH,
    MOVE_ROLL,
    MOVE_YAW,
    STOP_PITCH,
    STOP_ROLL,
    STOP_YAW,
    STOP_ALL,
    RC_MODE,
    RC_FIX_MODE,
    RC_NOROLL_MODE,
    GOTO_ZERO
};

inline uint8_t airtToAerotools(uint8_t airtCommand)
{
    return airtCommand - (airt::StdMessage::GIMBAL_GOTO_ANGLE - GOTO_ANGLE);
}

enum Notifications
{
};

// From Aerotools codes -> AiRT
// static unsigned char ATtoAirtCommands[]{
//     0,
//     StdMessage::FCS_PING,        // PING = 1,
//     StdMessage::FCS_VERSION,     // VERSION,
//     StdMessage::FCS_CLEARALL,    // CLEARALL,
//     StdMessage::FCS_SETMODE,     // SETMODE,
//     StdMessage::FCS_GOTO,      // GOTOWP,
//     StdMessage::FCS_WP,          // WP,
//     StdMessage::FCS_ARM,         // ARM,
//     StdMessage::FCS_DISARM,      // DISARM,
//     StdMessage::FCS_TAKEOFF,     // TAKEOFF,
//     StdMessage::FCS_LAND,        // LAND,
//     StdMessage::FCS_POSITIONON,  // POSITIONON,
//     StdMessage::FCS_POSITIONOFF, // POSITIONOFF,
//     StdMessage::FCS_SETSPEED,    // SETSPEED
//     StdMessage::FCS_CREATEMISSION
// };

AeroToolsGimbalMessageProcessor::AeroToolsGimbalMessageProcessor() : inbuffer(bufferCapacity), state(Status::FIRST_SYNC), discarded_bytes(0), payload_read_bytes(0)
{
    static_assert(static_cast<uint8_t>(GOTO_ZERO) == 0xd, "Have you changed the Command enums? Make sure ATtoAirtCommands still works");
    static_assert(static_cast<uint8_t>(StdMessage::GIMBAL_GOTO_ANGLE) - static_cast<uint8_t>(GOTO_ANGLE) ==
                      static_cast<uint8_t>(StdMessage::GIMBAL_GOTO_ZERO) - static_cast<uint8_t>(GOTO_ZERO),
                  "Have you changed the Command enum? Make sure ATtoAirtCommands still works");
}

bool AeroToolsGimbalMessageProcessor::ingest(unsigned char *data, unsigned int size)
{
    if (size + inbuffer.size() > inbuffer.capacity())
        return false;

    inbuffer.insert(inbuffer.end(), data, data + size);
    return true;
}

bool AeroToolsGimbalMessageProcessor::sendCommand(Serial &port, enum airt::GimbalMultiplexerCommandType c)
{
    uint8_t airtCommand = static_cast<uint8_t>(c);
    switch (airtCommand)
    {
    case StdMessage::GIMBAL_STOP_PITCH:
    case StdMessage::GIMBAL_STOP_ROLL:
    case StdMessage::GIMBAL_STOP_YAW:
    case StdMessage::GIMBAL_STOP_ALL:
    case StdMessage::GIMBAL_RC_MODE:
    case StdMessage::GIMBAL_RC_FIX_MODE:
    case StdMessage::GIMBAL_RC_NOROLL_MODE:
    case StdMessage::GIMBAL_GOTO_ZERO:
        return sendATCommand(port, airtToAerotools(airtCommand));
    default:
        return false;
    }
}

bool AeroToolsGimbalMessageProcessor::sendATCommand(Serial &port, unsigned char c)
{
    unsigned char msg[5]{SYNC1, SYNC2, 0x0, 0x0, 0x0};

    msg[ID_INDEX] = c;
    msg[MIN_MESSAGE_SIZE - 1] = msg[ID_INDEX];
    Log::info("Sending to Gimbal non-payload command {}", c);
    return port.write(msg, sizeof(msg)) == sizeof(msg);
}

bool AeroToolsGimbalMessageProcessor::sendATCommand(Serial &port, unsigned char c, unsigned char *payload, unsigned int size)
{
    unsigned int totalMsgSize = size + MIN_MESSAGE_SIZE;
    unsigned char buffer[MAX_MSG_LENGTH];
    buffer[0] = SYNC1;
    buffer[1] = SYNC2;
    buffer[ID_INDEX] = c;
    buffer[LENGTH_INDEX] = size;
    memcpy(buffer + LENGTH_INDEX + 1, payload, size);
    buffer[totalMsgSize - 1] = checksum(buffer);
    Log::info("Sending to Gimbal command {} ({} bytes)", c, totalMsgSize);
    return port.write(buffer, totalMsgSize) == totalMsgSize;
}

unsigned char AeroToolsGimbalMessageProcessor::checksum(unsigned char *msg)
{
    assert(msg[0] == SYNC1);
    assert(msg[1] == SYNC2);

    unsigned int suma = msg[ID_INDEX] + msg[LENGTH_INDEX];
    for (int i = 0; i < msg[LENGTH_INDEX]; i++)
    {
        suma += msg[i + LENGTH_INDEX + 1];
    }

    return (unsigned char)suma;
}

bool AeroToolsGimbalMessageProcessor::hasMessages()
{
    if (state == Status::DONE)
        return true;

    return parse();
}

bool AeroToolsGimbalMessageProcessor::popMessage(unsigned char *buffer, const unsigned int max, unsigned int &size)
{
    if (!hasMessages())
        return false;

    assert(message.size() < max);

    translateNotification(buffer, max, size);
    state = Status::FIRST_SYNC;
    message.clear();
    payload_read_bytes = 0;

    return true;
}

void AeroToolsGimbalMessageProcessor::restart()
{
    discarded_bytes += message.size();
    message.clear();
    state = Status::FIRST_SYNC;
    payload_read_bytes = 0;
}

bool AeroToolsGimbalMessageProcessor::parse()
{
    while (!inbuffer.empty() && state != Status::DONE)
    {
        unsigned char c = inbuffer[0];
        inbuffer.pop_front();
        message.push_back(c);
        switch (state)
        {
        case Status::FIRST_SYNC:
            if (c == SYNC1)
                state = Status::SECOND_SYNC;
            else
                restart();
            break;
        case Status::SECOND_SYNC:
            if (c == SYNC2)
                state = Status::ID;
            else
                restart();
            break;
        case Status::ID:
            state = Status::LENGTH;
            break;
        case Status::LENGTH:
            if (c == 0)
                state = Status::CHKSUM;
            else
            {
                state = Status::PAYLOAD;
                payload_read_bytes = 0;
            }
            break;
        case Status::PAYLOAD:
            payload_read_bytes++;
            if (payload_read_bytes >= message[LENGTH_INDEX])
            {
                state = Status::CHKSUM;
            }
            break;
        case Status::CHKSUM:
            state = Status::DONE;
            Log::info("Received msg from serial: {} ({} bytes)", message[ID_INDEX], message.size());
            break;
        case Status::DONE:
            assert(false);
            break;
        }
    }
    return state == Status::DONE;
}

bool AeroToolsGimbalMessageProcessor::sendGotoAngle(Serial &port, float pitch, float roll, float yaw, float speedRadPerSec)
{
    short int sipitch = static_cast<short int>((pitch * 180.0 / M_PI) + 0.5), 
        siroll = static_cast<short int>((roll * 180.0 / M_PI) + 0.5),
        siyaw = static_cast<short int>((yaw * 180.0 / M_PI) + 0.5);
    uint8_t bspeed = static_cast<uint8_t>((speedRadPerSec * 180.0 / M_PI) + 0.5f);

    uint8_t buffer[7];

    buffer[0] = sipitch >> 8;
    buffer[1] = sipitch & 0xff;
    buffer[2] = siroll >> 8;
    buffer[3] = siroll & 0xff;
    buffer[4] = siyaw >> 8;
    buffer[5] = siyaw & 0xff;
    buffer[6] = bspeed;

    return sendATCommand(port, airtToAerotools(StdMessage::GIMBAL_GOTO_ANGLE), buffer, sizeof(buffer));
}

template <typename C>
void copyMessage(C *msg, unsigned char *buffer, unsigned int &size)
{
    memcpy(buffer, msg, sizeof(C));
    size = sizeof(C);
}

void AeroToolsGimbalMessageProcessor::translateNotification(unsigned char *buffer, const unsigned int max, unsigned int &size)
{
    switch (message[ID_INDEX])
    {
        break;
    }
}
