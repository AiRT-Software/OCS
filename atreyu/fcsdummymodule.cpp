#include "fcsdummymodule.h"

#include "globalSettings.h"

#include <log.h>
#include <sstream>
#include <limits>

namespace airt
{
#include "interface/pozyxmodule.h"
}

using airt::FCSDummyModule;
using airt::Log;
using airt::StdMessage;
using airt::StdNotification;

FCSDummyModule::FCSDummyModule(const std::string &cmdportname, const std::string &pubportname, const std::string &subname,
                               std::shared_ptr<Context> context)
    : Module(cmdportname, pubportname, subname, context)
{
    assert(context);

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
    battery = 100;
    motors = 0;
    requestWP = 0;
    flyingMission = false;

    publishPeriodS = 0.5;  // 500 ms
    arrivedDistance = 0.5; // 0.5 m
}

void FCSDummyModule::configureSubscriptions(const std::string &mainpublisher)
{
    assert(subsocket);
    subsocket->connect(mainpublisher);
    uint8_t pozyxsignature[]{'A', StdMessage::POSITIONING_NOTIFICATIONS_MODULE};
    subsocket->subscribe(pozyxsignature, sizeof(pozyxsignature));
}

void FCSDummyModule::onNotification(const Message &m)
{
    assert(airt::getMessageModule(m) == StdMessage::POSITIONING_NOTIFICATIONS_MODULE);

    if (airt::getMessageAction(m) == StdMessage::IPS_DATA)
    {
        const PositioningFrame *pos = m.get<const PositioningFrame *>(0);

        glm::vec3 currentPos = glm::vec3(pos->x, pos->y, pos->z);
        if (flyingMission && motors > 90)
        {
            auto dest = waypoints[destinationWP];
            float d = glm::distance(currentPos, dest);
            if (d < arrivedDistance * 1000)
            {
                ++destinationWP;
                WaypointReachedNotification wr;
                wr.waypoint = destinationWP;
                airt::sendOnePartMessage(*pubsocket, &wr);
                if (destinationWP >= waypoints.size())
                {
                    destinationWP = waypoints.size() - 1; // we should receive the LAND command from the planexec
                }
            }
        }
    }
}

bool FCSDummyModule::startDevice()
{
    Log::info("FCS Dummy Module starts...");
    airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::STARTED);
    return true;
};

void FCSDummyModule::stopDevice()
{
    Log::info("FCS Dummy Module stops...");
    airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::STOPPED);
};

void FCSDummyModule::onMessage(const Message &inmsg)
{
    uint8_t command = airt::getMessageAction(inmsg);
    switch (command)
    {
    case StdMessage::FCS_PING:
        airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_PONG_NOTIFICATION);
        break;
    //case StdMessage::FCS_VERSION:
    case StdMessage::FCS_CLEARALL:
        // non-notified
        break;
    case StdMessage::FCS_ARM:
        motors = 50;
        //airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_ARMED_NOTIFICATION);
        break;
    case StdMessage::FCS_DISARM:
    case StdMessage::FCS_LAND:
        motors = 0;
        flyingMission = false;
        //airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_DISARMED_NOTIFICATION);
        break;
    case StdMessage::FCS_POSITIONON:
    case StdMessage::FCS_POSITIONOFF:
        break;
    case StdMessage::FCS_TAKEOFF:
    {
        motors = 100;
    }
    break;
    case StdMessage::FCS_SETMODE:
    {
        const SetModeFCS *sm = inmsg.get<const SetModeFCS *>(0);
        Log::info("Setting FCS to mode {}", sm->mode);
        if (sm->mode == StdMessage::MODE_AUTO)
        {
            flyingMission = true;
            destinationWP = 0;
        }
    }
    break;
    case StdMessage::FCS_GOTO:
    {
        const GotoLocation *gl = inmsg.get<const GotoLocation *>(0);
        Log::info("Sending the drone to {}, {}, {}", gl->x, gl->y, gl->z);
        //     messageProcessor.sendGoToCommand(port, gl->x, gl->y, gl->z);
    }
    break;
    case StdMessage::FCS_WP:
    {
        const UploadWaypoint *wp = inmsg.get<const UploadWaypoint *>(0);
        Log::info("Uploading waypoint {}: {}, {}, {}", wp->id, wp->x, wp->y, wp->z);
        waypoints.push_back(glm::vec3(wp->x, wp->y, wp->z));
        //   messageProcessor.sendAddWaypointCommand(port, wp->id, wp->x, wp->y, wp->z);
        // notify the wp in order to be catched by the pozyxDummy
        FCSUploadedWaypoint uwp;
        uwp.x = wp->x;
        uwp.y = wp->y;
        uwp.z = wp->z;
        uwp.id = wp->id;
        airt::sendOnePartMessage(*pubsocket, &uwp);

        if (wp->id == requestWP)
        {
            requestWP = 0;
            airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_MISSIONACK_NOTIFICATION);
        }
        else
        {
            RequestWaypointNotification r;
            r.waypoint = wp->id + 1;
            airt::sendOnePartMessage(*pubsocket, &r);
        }
    }
    break;
    case StdMessage::FCS_SETSPEED:
    {
        const SetFCSSpeed *ss = inmsg.get<const SetFCSSpeed *>(0);
        Log::info("Set speed to {}", ss->speed);

        SpeedChanged sc;
        sc.speed = ss->speed;
        airt::sendOnePartMessage(*pubsocket, &sc);
        break;
    }
    case StdMessage::FCS_CREATEMISSION:
    {
        const CreateMission *cm = inmsg.get<const CreateMission *>(0);
        requestWP = cm->numberOfWP;
        waypoints.clear();
        destinationWP = 0;

        RequestWaypointNotification r;
        r.waypoint = 1;
        airt::sendOnePartMessage(*pubsocket, &r);
        break;
    }
    default:
        Log::critical("Unhandled message in FCS module");
        airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::UNKNOWN_COMMAND);
        return;
    }
    airt::sendNotification(*pubsocket, StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::ACK);
}

bool FCSDummyModule::dataReady()
{
    static std::chrono::time_point<std::chrono::steady_clock> prevNotif = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - prevNotif;
    if (elapsed.count() >= publishPeriodS)
    {
        prevNotif = now;
        return true;
    }
    else
        return false;
}

bool FCSDummyModule::sendMessage()
{

    MotorSpeedNotification ms;
    ms.power = motors;
    airt::sendOnePartMessage(*pubsocket, &ms);

    BatteryLevelNotification bl;
    if (motors > 0)
        --battery;
    bl.level = battery;
    airt::sendOnePartMessage(*pubsocket, &bl);
    return true;
}
