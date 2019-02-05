#include "fcsmodule.h"

#include "globalSettings.h"

#include <log.h>
#include <sstream>
#include <limits>

using airt::FCSModule;
using airt::Log;
using airt::StdMessage;
using airt::StdNotification;

FCSModule::FCSModule(const std::string &cmdportname, const std::string &pubportname,
                     std::shared_ptr<Context> context)
    : Module(cmdportname, pubportname, "", context)
{
    assert(context);

    if (!GlobalSettings::getValue("fcs_uart_port", serialport))
    {
        serialport = "/dev/ttyUSB0";
        Log::warn("Using default value fcs_uart_port = {}", serialport);
    }

    if (!GlobalSettings::getValue("fcs_baud_rate", baudrate))
    {
        baudrate = 115200;
        Log::warn("Using default value fcs_baud_rate = {}", baudrate);
    }

    double tlon, tlat;
    if (!GlobalSettings::getValue("fcs_origin_lon", tlon))
    {
        tlon = 5;
        Log::warn("Using default value for fcs_origin_lon: {} degrees", tlon);
    }

    if (!GlobalSettings::getValue("fcs_origin_lat", tlat))
    {
        tlat = 50;
        Log::warn("Using default value for pozyx2gps_origin_lat: {} degrees", tlat);
    }
    messageProcessor.setGPSOrigin(tlon, tlat);
}

bool FCSModule::startDevice()
{
    Log::info("FCSModule starts...");
    if (!port.isOpen() && !port.open(serialport, baudrate))
    {
        Log::critical("Failed to open serial port {} at {} bauds", serialport, baudrate);
        throw new std::runtime_error("Error opening serial port");
    }
    airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::STARTED);
    return true;
};

void FCSModule::stopDevice()
{
    if (port.isOpen())
        port.close();
    Log::info("FCSModule stops...");
    airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::STOPPED);
};

bool FCSModule::dataReady()
{
    unsigned char buffer[64];
    size_t bytesread;
    while ((bytesread = port.read(buffer, sizeof(buffer))) > 0)
    {
        messageProcessor.ingest(buffer, bytesread);
        if (bytesread < sizeof(buffer))
            break;
    }
    return messageProcessor.hasMessages();
}

void FCSModule::onMessage(const Message &inmsg)
{
    uint8_t command = airt::getMessageAction(inmsg);
    switch (command)
    {
    case StdMessage::FCS_PING:
    case StdMessage::FCS_VERSION:
    case StdMessage::FCS_CLEARALL:
    case StdMessage::FCS_ARM:
    case StdMessage::FCS_DISARM:
    case StdMessage::FCS_LAND:
    case StdMessage::FCS_POSITIONON:
    case StdMessage::FCS_POSITIONOFF:
        Log::info("Non payload command to FCS: {}", command);
        messageProcessor.sendCommand(port, static_cast<airt::FCSMultiplexerCommandType>(command));
        break;
    case StdMessage::FCS_TAKEOFF:
    {
        const TakeOff *to = inmsg.get<const TakeOff *>(0);
        Log::info("Taking off to {} m", to->height);
        messageProcessor.sendTakeOffCommand(port, to->height);
    }
    break;
    case StdMessage::FCS_SETMODE:
    {
        const SetModeFCS *sm = inmsg.get<const SetModeFCS *>(0);
        Log::info("Setting FCS to mode {}", sm->mode);
        messageProcessor.sendSetModeCommand(port, sm->mode);
    }
    break;
    case StdMessage::FCS_GOTO:
    {
        const GotoLocation *gl = inmsg.get<const GotoLocation *>(0);
        Log::info("Sending the drone to {}, {}, {}", gl->x, gl->y, gl->z);
        messageProcessor.sendGoToCommand(port, gl->x, gl->y, gl->z);
    }
    break;
    case StdMessage::FCS_WP:
    {
        const UploadWaypoint *wp = inmsg.get<const UploadWaypoint *>(0);
        Log::info("Uploading waypoint {}: {}, {}, {}", wp->id, wp->x, wp->y, wp->z);
        messageProcessor.sendAddWaypointCommand(port, wp->id, wp->x, wp->y, wp->z);

        // notify the wp in order to be catched by the pozyxDummy
        FCSUploadedWaypoint uwp;
        uwp.x = wp->x;
        uwp.y = wp->y;
        uwp.z = wp->z;
        uwp.id = wp->id;
        airt::sendOnePartMessage(*pubsocket, &uwp);
    }
    break;
    case StdMessage::FCS_SETSPEED:
    {
        const SetFCSSpeed *ss = inmsg.get<const SetFCSSpeed *>(0);
        messageProcessor.sendSetSpeed(port, ss->speed);

        SpeedChanged sc;
        sc.speed = ss->speed;
        airt::sendOnePartMessage(*pubsocket, &sc);
    }
    break;
    case StdMessage::FCS_CREATEMISSION:
    {
        const CreateMission *cm = inmsg.get<const CreateMission *>(0);
        messageProcessor.sendCreateMission(port, cm->numberOfWP);
    }
    break;
    default:
        Log::critical("Unhandled message in FCS module");
        airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::UNKNOWN_COMMAND);
        return;
    }
    airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::ACK);
}

bool FCSModule::sendMessage()
{
    while (messageProcessor.hasMessages())
    {
        unsigned char buffer[128];
        unsigned int size;
        if (messageProcessor.popMessage(buffer, sizeof(buffer), size))
        {
            Message msg;
            msg.add_raw(buffer, size);
            pubsocket->send(msg);
        }
        else
        {
            Log::warn("Supposedly there was a message waiting from the fcs serial port");
        }
    }
    return true;
}
