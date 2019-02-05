
#include <cstring>
#include <cassert>

#include "../atreyu/include/fcsmodule.h"
#include "aerotoolsFCSMessageProcessor.h"
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

using airt::AeroToolsFCSMessageProcessor;
using airt::StdMessage;

enum Commands
{
    PING = 1,
    VERSION_DEPRECATED,
    CLEARALL,
    SETMODE,
    GOTO,
    WP,
    ARM,
    DISARM,
    TAKEOFF,
    LAND,
    POSITIONON,
    POSITIONOFF,
    SETSPEED,
    CREATEMISSION
};

enum Notifications
{
    MISSION_ACK = 0xf5,
    WP_REQUEST,
    VERSION_RESPONSE_DEPRECATED,
    WPREACHED,
    ROLLPITCHYAW,
    MOTORS,
    BATTERY,
    PONG,
    DATAPOS,
    NACK,
    ACK
};

// From Aerotools codes -> AiRT
static unsigned char ATtoAirtCommands[]{
    0,
    StdMessage::FCS_PING,        // PING = 1,
    StdMessage::FCS_VERSION,     // VERSION,
    StdMessage::FCS_CLEARALL,    // CLEARALL,
    StdMessage::FCS_SETMODE,     // SETMODE,
    StdMessage::FCS_GOTO,        // GOTOWP,
    StdMessage::FCS_WP,          // WP,
    StdMessage::FCS_ARM,         // ARM,
    StdMessage::FCS_DISARM,      // DISARM,
    StdMessage::FCS_TAKEOFF,     // TAKEOFF,
    StdMessage::FCS_LAND,        // LAND,
    StdMessage::FCS_POSITIONON,  // POSITIONON,
    StdMessage::FCS_POSITIONOFF, // POSITIONOFF,
    StdMessage::FCS_SETSPEED,    // SETSPEED
    StdMessage::FCS_CREATEMISSION};

inline uint8_t airtToAerotools(uint8_t airtCommand)
{
    return airtCommand - (airt::StdMessage::FCS_PING - PING);
}

AeroToolsFCSMessageProcessor::AeroToolsFCSMessageProcessor() : pozyx2gps_origin_lon(0.0f),
                                                               pozyx2gps_origin_lat(0.0f),
                                                               inbuffer(bufferCapacity),
                                                               state(Status::FIRST_SYNC),
                                                               discarded_bytes(0),
                                                               payload_read_bytes(0)
{
    static_assert(static_cast<uint8_t>(SETSPEED) == 0xd, "Have you changed the Command enum? Make sure ATtoAirtCommands still works");
    static_assert(static_cast<uint8_t>(StdMessage::FCS_PING) - static_cast<uint8_t>(PING) ==
                      static_cast<uint8_t>(StdMessage::FCS_CREATEMISSION) - static_cast<uint8_t>(CREATEMISSION),
                  "Have you changed the Command enum? Make sure ATtoAirtCommands still works");
}

bool AeroToolsFCSMessageProcessor::ingest(unsigned char *data, unsigned int size)
{
    if (size + inbuffer.size() > inbuffer.capacity())
        return false;

    inbuffer.insert(inbuffer.end(), data, data + size);
    return true;
}

bool AeroToolsFCSMessageProcessor::sendCommand(Serial &port, enum airt::FCSMultiplexerCommandType c)
{
    switch (static_cast<uint8_t>(c))
    {
    case StdMessage::FCS_PING:
        return sendATCommand(port, PING);
        break;
    // case StdMessage::FCS_VERSION:
    //     Log::error("FCS_VERSION command has been deprecated");
    //     //return sendATCommand(port, VERSION);
    //     break;
    case StdMessage::FCS_CLEARALL:
        return sendATCommand(port, CLEARALL);
        break;
    case StdMessage::FCS_ARM:
        return sendATCommand(port, ARM);
        break;
    case StdMessage::FCS_DISARM:
        return sendATCommand(port, DISARM);
        break;
    case StdMessage::FCS_LAND:
        return sendATCommand(port, LAND);
        break;
    case StdMessage::FCS_POSITIONON:
        return sendATCommand(port, POSITIONON);
        break;
    case StdMessage::FCS_POSITIONOFF:
        return sendATCommand(port, POSITIONOFF);
        break;
    default:
        return false;
    }
}

bool AeroToolsFCSMessageProcessor::sendATCommand(Serial &port, unsigned char c)
{
    unsigned char msg[5]{SYNC1, SYNC2, 0x0, 0x0, 0x0};

    msg[ID_INDEX] = c;
    msg[MIN_MESSAGE_SIZE - 1] = msg[ID_INDEX];
    Log::info("Sending to fcs non-payload command {}", c);
    return port.write(msg, sizeof(msg)) == sizeof(msg);
}

bool AeroToolsFCSMessageProcessor::sendATCommand(Serial &port, unsigned char c, const void *payload, unsigned int size)
{
    unsigned int totalMsgSize = size + MIN_MESSAGE_SIZE;
    unsigned char buffer[MAX_MSG_LENGTH];
    buffer[0] = SYNC1;
    buffer[1] = SYNC2;
    buffer[ID_INDEX] = c;
    buffer[LENGTH_INDEX] = size;
    memcpy(buffer + LENGTH_INDEX + 1, payload, size);
    buffer[totalMsgSize - 1] = checksum(buffer);
    Log::info("Sending to FCS command {} ({} bytes)", c, totalMsgSize);
    return port.write(buffer, totalMsgSize) == totalMsgSize;
}

unsigned char AeroToolsFCSMessageProcessor::checksum(unsigned char *msg)
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

bool AeroToolsFCSMessageProcessor::sendSetModeCommand(Serial &port, uint8_t mode)
{
    return sendATCommand(port, SETMODE, &mode, 1);
}

bool AeroToolsFCSMessageProcessor::sendGoToCommand(Serial &port, float x, float y, float height)
{
    int32_t gpsx, gpsy, gpsz;
    pozyx2gps(x, y, height, gpsx, gpsy, gpsz);

    uint8_t buffer[sizeof(int32_t) * 3];
    writeBytesHighToLow(buffer, gpsx);
    writeBytesHighToLow(buffer + 4, gpsy);
    writeBytesHighToLow(buffer + 8, gpsz);

    return sendATCommand(port, GOTO, buffer, sizeof(buffer));
}

bool AeroToolsFCSMessageProcessor::sendAddWaypointCommand(Serial &port, uint16_t id, float x, float y, float height)
{
    int32_t gpsx, gpsy, gpsz;
    pozyx2gps(x, y, height, gpsx, gpsy, gpsz);

    uint8_t buffer[sizeof(uint16_t) + sizeof(int32_t) * 3];
    writeBytesHighToLow(buffer, id);
    writeBytesHighToLow(buffer + sizeof(uint16_t), gpsx);
    writeBytesHighToLow(buffer + sizeof(uint16_t) + sizeof(int32_t), gpsy);
    writeBytesHighToLow(buffer + sizeof(uint16_t) + 2 * sizeof(int32_t), gpsz);
    return sendATCommand(port, WP, buffer, sizeof(buffer));
}

bool AeroToolsFCSMessageProcessor::sendTakeOffCommand(Serial &port, float height_m)
{
    int32_t hm = static_cast<int32_t>(height_m);
    uint8_t buffer[4];

    writeBytesHighToLow(buffer, hm);
    return sendATCommand(port, TAKEOFF, buffer, sizeof(buffer));
}

bool AeroToolsFCSMessageProcessor::sendSetSpeed(Serial &port, uint8_t speed)
{
    return sendATCommand(port, SETSPEED, &speed, 1);
}

bool AeroToolsFCSMessageProcessor::sendCreateMission(Serial &port, uint16_t numWPs)
{
    uint8_t buffer[2];
    writeBytesHighToLow(buffer, numWPs);
    return sendATCommand(port, CREATEMISSION, buffer, sizeof(buffer));
}

bool AeroToolsFCSMessageProcessor::hasMessages()
{
    if (state == Status::DONE)
        return true;

    return parse();
}

bool AeroToolsFCSMessageProcessor::popMessage(unsigned char *buffer, const unsigned int max, unsigned int &size)
{
    if (!hasMessages())
        return false;

    if (message.size() > max)
    {
        Log::error("Message too large: {} bytes. Discarding", message.size());
        restart();
        return false;
    }
    else
    {
        translateNotification(buffer, max, size);
        state = Status::FIRST_SYNC;
        message.clear();
        payload_read_bytes = 0;

        return true;
    }
}

void AeroToolsFCSMessageProcessor::restart()
{
    discarded_bytes += message.size();
    message.clear();
    state = Status::FIRST_SYNC;
    payload_read_bytes = 0;
}

bool AeroToolsFCSMessageProcessor::parse()
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

template <typename C>
void copyMessage(C *msg, unsigned char *buffer, unsigned int &size)
{
    memcpy(buffer, msg, sizeof(C));
    size = sizeof(C);
}

void AeroToolsFCSMessageProcessor::translateNotification(unsigned char *buffer, const unsigned int max, unsigned int &size)
{
    switch (message[ID_INDEX])
    {
    case MISSION_ACK:
    {
        airt::StdMessage m(StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_MISSIONACK_NOTIFICATION);
        copyMessage(&m, buffer, size);
    }
    break;
    case WP_REQUEST:
    {
        airt::FCSModule::RequestWaypointNotification n;
        if (message[LENGTH_INDEX] == 1)
        {
            n.waypoint = message[PAYLOAD_INDEX];
        }
        else
        {
            airt::readBytesHighToLow(&message[PAYLOAD_INDEX], n.waypoint);
        }
        copyMessage(&n, buffer, size);
    }
    break;
    // case VERSION_RESPONSE:
    //     assert(false);
    //     break;
    case WPREACHED:
    {
        airt::FCSModule::WaypointReachedNotification m;
        m.waypoint = (message[PAYLOAD_INDEX] << 8) + message[PAYLOAD_INDEX + 1];
        copyMessage(&m, buffer, size);
    }
    break;
    case ROLLPITCHYAW:
    {
        airt::FCSModule::RollPitchYawNotification m;
        int16_t tmp;
        airt::readBytesHighToLow(&message[PAYLOAD_INDEX + 4], tmp);
        m.roll = static_cast<float>(tmp) * M_PI / (180.0 * 10.0);
        airt::readBytesHighToLow(&message[PAYLOAD_INDEX + 2], tmp);
        m.pitch = static_cast<float>(tmp) * M_PI / (180.0 * 10.0);
        airt::readBytesHighToLow(&message[PAYLOAD_INDEX], tmp);
        m.yaw = static_cast<float>(tmp) * M_PI / (180.0 * 10.0);
        copyMessage(&m, buffer, size);
    }
    break;
    case MOTORS:
    {
        airt::FCSModule::MotorSpeedNotification m;
        m.power = message[PAYLOAD_INDEX];
        copyMessage(&m, buffer, size);
    }
    break;
    case BATTERY:
    {
        airt::FCSModule::BatteryLevelNotification m;
        m.level = message[PAYLOAD_INDEX];
        copyMessage(&m, buffer, size);
    }
    break;
    case PONG:
    {
        airt::StdMessage m(StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_PONG_NOTIFICATION);
        copyMessage(&m, buffer, size);
    }
    break;
    case DATAPOS:
    {
        airt::FCSModule::FCSPositionNotification m;
        int32_t gpsx, gpsy, gpsz;

        airt::readBytesHighToLow(&message[PAYLOAD_INDEX], gpsx);
        airt::readBytesHighToLow(&message[PAYLOAD_INDEX + 4], gpsy);
        airt::readBytesHighToLow(&message[PAYLOAD_INDEX + 8], gpsz);

        if (gpsx == 0 && gpsy == 0)
        {
            // There is no gps. We will assume we are at the fake origin
            gpsx = pozyx2gps_origin_lon * 10000000;
            gpsy = pozyx2gps_origin_lat * 10000000;
        }
        gps2pozyx(gpsx, gpsy, gpsz, m.x, m.y, m.z);
        copyMessage(&m, buffer, size);
    }
    break;
    case NACK:
    {
        airt::FCSModule::NackNotification m;
        assert(message[PAYLOAD_INDEX] <= airt::arraySize(ATtoAirtCommands));
        m.command = ATtoAirtCommands[message[PAYLOAD_INDEX]];
        copyMessage(&m, buffer, size);
    }
    break;
    case ACK:
    {
        airt::FCSModule::AckNotification m;
        assert(message[PAYLOAD_INDEX] <= airt::arraySize(ATtoAirtCommands));
        m.command = ATtoAirtCommands[message[PAYLOAD_INDEX]];
        copyMessage(&m, buffer, size);
    }
    break;
    }
}

constexpr double EARTH_RADIUS_METERS = 6378137.0;

void AeroToolsFCSMessageProcessor::pozyx2gps(float x_mm, float y_mm, float z_mm, int32_t &lon_deg, int32_t &lat_deg, int32_t &h_mm)
{
    // https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters/2980#2980

    // offsets in meters (North and East)
    // https://en.wikipedia.org/wiki/Rotation_of_axes#Method_1

    double x_m = x_mm / 1000.0;
    double y_m = y_mm / 1000.0;

    // Coordinate offsets
    double delta_longitude = (x_m / (EARTH_RADIUS_METERS * cos(M_PI * pozyx2gps_origin_lat / 180.0))) * 180.0 / M_PI;
    double delta_latitude = (y_m / EARTH_RADIUS_METERS) * 180.0 / M_PI;

    // OffsetPosition, decimal degrees
    lat_deg = static_cast<int32_t>((pozyx2gps_origin_lat + delta_latitude) * 1e7);
    lon_deg = static_cast<int32_t>((pozyx2gps_origin_lon + delta_longitude) * 1e7);
    h_mm = static_cast<int32_t>(z_mm);
}

void AeroToolsFCSMessageProcessor::gps2pozyx(int32_t lon_deg, int32_t lat_deg, int32_t h_mm, float &x_mm, float &y_mm, float &z_mm)
{
    double lond = lon_deg * 1e-7 - pozyx2gps_origin_lon;
    double latd = lat_deg * 1e-7 - pozyx2gps_origin_lat;

    // Coordinate offsets
    double x_m = (EARTH_RADIUS_METERS * cos(M_PI * pozyx2gps_origin_lat / 180.0)) * lond / (180.0 / M_PI);
    double y_m = (EARTH_RADIUS_METERS * latd) / (180.0 / M_PI);

    x_mm = x_m * 1000.0;
    y_mm = y_m * 1000.0;
    z_mm = h_mm;
}
