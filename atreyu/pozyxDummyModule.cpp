
#include "pozyxDummyModule.h"
#include "globalSettings.h"

#include <log.h>
#include <stdMessage.h>
#include <glm/gtc/random.hpp>

namespace airt
{
#include "interface/fcsmodule.h"
};

using airt::Context;
using airt::GlobalSettings;
using airt::Log;
using airt::Message;
using airt::PozyxDummyModule;

PozyxDummyModule::PozyxDummyModule(const std::string &cmdportname, const std::string &pubportname, const std::string &subportname,
                                   std::shared_ptr<Context> context) : Module(cmdportname, pubportname, subportname, context)
{
    assert(context);

    errorStdDev = glm::vec3(0.0f);
    moving = false;
    publishPeriodS = 0.1; // 10 Hz
    currentFrame = 0;
    currentPos = glm::vec3(1000.f, 1000.f, 200.f);
    speedMeterPerSec = 0.4;

    if (!GlobalSettings::getValue("pozyx_settings_file", pozyx_settingsfile_path))
    {
        Log::critical("PozyxDummyModule: Undefined pozyx_settings_file");
        throw std::runtime_error("PozyxDummyModule: Undefined pozyx_settings_file");
    }

    ips_settings_updated = false;
}

void PozyxDummyModule::configureSubscriptions(const std::string &mainpublisher)
{
    assert(subsocket);
    subsocket->connect(mainpublisher);
    uint8_t fcssignature[]{'A', StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE};
    subsocket->subscribe(fcssignature, sizeof(fcssignature));
    uint8_t planexecsignature[]{'A', StdMessage::PLAN_EXECUTOR_NOTIFICATIONS_MODULE};
    subsocket->subscribe(planexecsignature, sizeof(planexecsignature));
}

void PozyxDummyModule::onNotification(const Message &m)
{
    auto module = airt::getMessageModule(m);
    auto action = airt::getMessageAction(m);

    if (module == StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE)
    {
        switch (action)
        {
        case StdMessage::FCS_UPLOADED_WAYPOINT_NOTIFICATION:
        {
            const FCSUploadedWaypoint *uw = m.get<const FCSUploadedWaypoint *>(0);
            Log::info("PozyxDummyModule: received waypoint #{} upload notification", uw->id);
            auto wp = glm::vec3(uw->x, uw->y, uw->z);
            if (uw->id == 1)
            {
                // First: clear our list and add the current position
                waypoints.clear();
                currentPos = wp + glm::gaussRand(glm::vec3(0.0f), glm::vec3(2.0f));
                nextWaypoint = 0;
            }
            waypoints.push_back(wp);
            break;
        }
        case StdMessage::FCS_WPREACHED_NOTIFICATION:
        {
            const WaypointReachedNotification *rwp = m.get<const WaypointReachedNotification *>(0);
            Log::info("PozyxDummyModule: waypoint #{} reached", rwp->waypoint);
            nextWaypoint = rwp->waypoint;
            break;
        }
        case StdMessage::FCS_SPEED_CHANGED_NOTIFICATION:
        {
            const SpeedChanged *sc = m.get<const SpeedChanged *>(0);
            Log::info("PozyxDummyModule: new speed {} m/s", sc->speed);
            speedMeterPerSec = sc->speed;
            break;
        }
        }
    }
    else if (module == StdMessage::PLAN_EXECUTOR_NOTIFICATIONS_MODULE)
    {
        switch (action)
        {
        case StdMessage::PLAN_EXEC_LAUNCHED_NOTIFICATION:
            Log::info("PozyxDummyModule: launched notification");
            moving = true;
            break;
        case StdMessage::PLAN_EXEC_LANDING_NOTIFICATION:
            Log::info("PozyxDummyModule: land notification");
            moving = false;
            break;
        }
    }
}

bool PozyxDummyModule::startDevice()
{
    return true;
}

void PozyxDummyModule::stopDevice()
{
}

bool PozyxDummyModule::dataReady()
{
    if (!ips_settings_updated)
        return false;

    static std::chrono::time_point<std::chrono::steady_clock> lastPosFrameTS;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - lastPosFrameTS;
    if (elapsed.count() > publishPeriodS)
    {
        lastPosFrameTS = now;
        return true;
    }
    return false;
}

bool PozyxDummyModule::quitDevice()
{
    return true;
}

bool PozyxDummyModule::sendMessage()
{
    glm::vec3 wp;
    static double pozyxClock = 0.0, angleClock;
    if (!moving)
    {
        wp = currentPos;
    }
    else
    {
        glm::vec3 direction = glm::normalize(waypoints[nextWaypoint] - currentPos);
        currentPos += direction * publishPeriodS * speedMeterPerSec * 1000.0f;
        wp = currentPos;
        angleClock += publishPeriodS;
    }

    wp += glm::gaussRand(glm::vec3(0.0f), errorStdDev);

    PositioningFrame posdata;

    pozyxClock += publishPeriodS;

    posdata.timestamp = pozyxClock;
    posdata.x = wp.x;
    posdata.y = wp.y;
    posdata.z = wp.z;
    posdata.roll = glm::radians(20.0) * sin(angleClock * 2 * M_PI / 4.0);  // between +20 and -20 every 4 s
    posdata.pitch = glm::radians(30.0) * sin(angleClock * 2 * M_PI / 6.0); // between +30 and -30 every 6s
    posdata.yaw = glm::radians(fmod(angleClock * 2.0 / 10, 360.0));        // spinning around 36 degrees per second

    Message msg;
    msg.add_raw(&posdata, sizeof(PositioningFrame));
    pubsocket->send(msg);

    Log::info("PozyxDummyModule: frame {} roll {} pitch {} yaw {} x {} y {} z {}",
              currentFrame,
              posdata.roll, posdata.pitch, posdata.yaw,
              posdata.x, posdata.y, posdata.z);

    // update
    currentFrame++;
    return true;
}

void PozyxDummyModule::onMessage(const Message &m)
{

    switch (airt::getMessageAction(m))
    {
    case StdMessage::IPS_SYSTEM:
        Log::info("PozyxDummyModule: received command IPS_SYSTEM");
        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LOCALTAG_CONNECTED);

        break;

    case StdMessage::IPS_DISCOVER:
    {
        Log::info("PozyxDummyModule: received command IPS_DISCOVER");
        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_DISCOVERED);

        // send virtual ids and coordinates
        messageProcessor.loadSettings(pozyx_settingsfile_path, settings);
        sendAnchorsIDsList(settings);
        sendAnchorsLocations(settings);
    }
    break;

    case StdMessage::IPS_ANCHOR_MANUAL_CONFIG:
    {
        const PositioningAnchorManualConfig *am = m.get<const PositioningAnchorManualConfig *>(0);
        anchors_manual.push_back(*am);

        Log::info("PozyxDummyModule: received IPS_ANCHOR_MANUAL_CONFIG: id {} order {} x {} y {} z {}",
                  am->id, am->order, am->x, am->y, am->z);

        if (anchors_manual.size() >= 8)
        {
            for (size_t i = 0; i < anchors_manual.size(); i++)
            {
                PositioningAnchorManualConfig &a = anchors_manual[i];
                messageProcessor.setAnchorConfig(a.id, a.order, a.x, a.y, a.z, settings, resp_discover.anchors);
            }

            if (!messageProcessor.checkSettings(settings))
            {
                airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
                break;
            }

            // backup
            messageProcessor.saveSettings(pozyx_settingsfile_path + ".bak", settings);

            // override file
            messageProcessor.saveSettings(pozyx_settingsfile_path, settings);

            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_ANCHORS_MANUAL_CONFIG_ACCEPTED);
            anchors_manual.clear();
        }
    }
    break;

    case StdMessage::IPS_ANCHOR_TOBE_AUTOCALIBRATED:
    {
        const PositioningAnchorTobeAutocalibrated *am = m.get<const PositioningAnchorTobeAutocalibrated *>(0);

        PozyxPayloads::AutocalibAnchorConfig aac;
        aac.id = std::to_string(am->id);
        aac.order = am->order;
        aac.coordinates.x = am->x;
        aac.coordinates.y = am->y;
        aac.coordinates.z = am->z;
        anchors_tobe_autocalibrated.push_back(aac);

        Log::info("PozyxDummyModule: received IPS_ANCHOR_TOBE_AUTOCALIBRATED: id {} order {} x {} y {} z {}",
                  am->id, am->order, am->x, am->y, am->z);

        if (anchors_tobe_autocalibrated.size() >= 8)
        {
            messageProcessor.buildAutocalib(anchors_tobe_autocalibrated, autocalib);

            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_ANCHORS_TOBE_AUTOCALIBRATED_ACCEPTED);
            anchors_tobe_autocalibrated.clear();
        }
    }
    break;

    case StdMessage::IPS_AUTOCALIBRATE:
        Log::info("PozyxDummyModule: received command: IPS_AUTOCALIBRATE");

        if (autocalib.anchors.size() == 0)
        {
            Log::error("PozyxDummyModule: autocalibration payload does not have any anchor");
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            break;
        }

        if (messageProcessor.getSyntheticAutocalibrateResponse(autocalib, resp_autocalibrate))
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_AUTOCALIBRATED);

            Log::info("PozyxDummyModule: got autocalibrate response: succes {} score {} connectivity {}",
                      resp_autocalibrate.success, resp_autocalibrate.score, resp_autocalibrate.connectivity);

            sendAnchorsLocations(resp_autocalibrate);
        }

        break;

    case StdMessage::IPS_UPDATE_SETTINGS:
    {
        Log::info("PozyxDummyModule: received command IPS_UPDATE_SETTINGS");
        if (settings.anchors.size() == 0)
        {
            if (!messageProcessor.loadSettings(pozyx_settingsfile_path, settings))
            {
                airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
                Log::error("PozyxDummyModule: sent IPS_LAST_REQUEST_HAS_FAILED");
                break;
            }
        }

        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_UPDATED_SETTINGS);
        ips_settings_updated = true;
        sendAnchorsLocations(settings);
    }
    break;

    case StdMessage::IPS_START_POSITIONING:
    {
        Log::info("PozyxDummyModule: received command IPS_START_POSITIONING");
        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_POSITIONING_STARTED);

        // reset
        currentFrame = 0;
        Log::info("PozyxDummyModule: starting... Current frame: {}", currentFrame);
    }
    break;
    case StdMessage::IPS_STOP_POSITIONING:
    {
        Log::info("PozyxDummyModule: received command IPS_STOP_POSITIONING");
        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_POSITIONING_STOPPED);
        Log::info("PozyxDummyModule: stopping... Current frame: {}", currentFrame);
        ips_settings_updated = false;
    }
    break;
    default:
        Log::error("PozyxDummyModule: Unknown command: {}", airt::to_string(m));
        break;
    }

    return;
}

bool PozyxDummyModule::sendAnchorsIDsList(const PozyxPayloads::Settings &settings)
{
    Message msg;
    positioningAnchorsListHdr listHdr;
    std::vector<positioningAnchorsListItem> items;

    listHdr.numAnchors = settings.anchors.size();
    msg.add_raw(&listHdr, sizeof(listHdr));

    for (size_t i = 0; i < settings.anchors.size(); i++)
    {
        positioningAnchorsListItem item;
        item.id = std::stoi(settings.anchors[i].id);
        items.push_back(item);
    }

    msg.add_raw(&items[0], sizeof(positioningAnchorsListItem) * items.size());

    pubsocket->send(msg);
    Log::info("PozyxDummyModule: sending anchors list. Num anchors: {}", items.size());

    return true;
}

bool PozyxDummyModule::sendAnchorsLocations(const PozyxResponses::Autocalibrate &autocalibrate)
{
    for (size_t i = 0; i < autocalibrate.anchors.size(); i++)
    {
        Message msg;
        positioningAnchorLocationHeader locHdr;
        positioningAnchorLocation loc;

        loc.id = std::atoi(autocalibrate.anchors[i].id.c_str());
        loc.order = autocalibrate.anchors[i].order;
        loc.x = autocalibrate.anchors[i].coordinates.x;
        loc.y = autocalibrate.anchors[i].coordinates.y;
        loc.z = autocalibrate.anchors[i].coordinates.z;

        msg.add_raw(&locHdr, sizeof(locHdr));
        msg.add_raw(&loc, sizeof(loc));

        pubsocket->send(msg);
        Log::info("PozyxDummyModule: sending anchor location from autocalibrate: id {} order {} x {} y {} z {}",
                  loc.id, loc.order, loc.x, loc.y, loc.z);
    }

    return true;
}

bool PozyxDummyModule::sendAnchorsLocations(const PozyxPayloads::Settings &settings)
{
    Message msg;
    positioningAnchorLocationHeader locHdr;
    locHdr.numAnchors = settings.anchors.size();

    msg.add_raw(&locHdr, sizeof(locHdr));

    for (size_t i = 0; i < settings.anchors.size(); i++)
    {
        positioningAnchorLocation loc;

        loc.id = std::stoi(settings.anchors[i].id);

        /*
        Note:
        anchors in settings must be organized like this:
          first the 4 anchors at the bottom (origin, x, corner, y), order[0,3]
          second the 4 anchors at the top (origin-top, x-top, corner-top, y-top), order[4,7]
          Then we can get the order directly from the anchors array of the settings file
        */
        loc.order = i;

        loc.x = settings.anchors[i].coordinates.x;
        loc.y = settings.anchors[i].coordinates.y;
        loc.z = settings.anchors[i].coordinates.z;

        msg.add_raw(&loc, sizeof(loc));

        Log::info("PozyxModule: sending anchor location from settings: id {} order {} x {} y {} z {}",
                  loc.id, loc.order, loc.x, loc.y, loc.z);
    }

    pubsocket->send(msg);
    return true;
}
