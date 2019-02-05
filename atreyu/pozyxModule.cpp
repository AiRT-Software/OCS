
#include "pozyxModule.h"
#include "globalSettings.h"

#include <log.h>
#include <stdMessage.h>

using airt::Context;
using airt::GlobalSettings;
using airt::Log;
using airt::Message;
using airt::PozyxModule;

using boost::asio::ip::tcp;
using boost::asio::ip::udp;

#include <utils.h>

PozyxModule::PozyxModule(const std::string &cmdportname, const std::string &subportname, std::shared_ptr<Context> context) : Module(cmdportname, subportname, "", context)
{
    assert(context);

    std::string pozyx_ip = "";
    if (!GlobalSettings::getValue("pozyx_ip", pozyx_ip))
    {
        Log::critical("PozyxModule: Undefined pozyx_ip");
        throw std::runtime_error("PozyxModule: Undefined pozyx_ip");
    }

    int pozyx_httpport;
    if (!GlobalSettings::getValue("pozyx_httpport", pozyx_httpport))
    {
        Log::critical("PozyxModule: Undefined pozyx_httpport");
        throw std::runtime_error("PozyxModule: Undefined pozyx_httpport");
    }

    messageProcessor.setEndpoints(pozyx_ip, std::to_string(pozyx_httpport));

    if (!GlobalSettings::getValue("pozyx_udpport", pozyx_udpport))
    {
        Log::critical("PozyxModule: Undefined pozyx_udpport");
        throw std::runtime_error("PozyxModule: Undefined pozyx_udpport");
    }

    if (!GlobalSettings::getValue("pozyx_settings_file", pozyx_settingsfile_path))
    {
        Log::critical("PozyxModule: Undefined pozyx_settings_file");
        throw std::runtime_error("PozyxModule: Undefined pozyx_settings_file");
    }

    // udp socket will attend at pozyx-server udp port
    udp_socket = std::make_unique<udp::socket>(udp_io_service, udp::endpoint(udp::v4(), pozyx_udpport));

    // close the tap to avoid the o.s. enqueues unrequired data
    udp_socket->close();

    // init
    currentFrame = 0;
    max_secs_for_data = 1.5;
    ips_settings_updated = false;
}

bool PozyxModule::startDevice()
{
    return true;
}

void PozyxModule::stopDevice()
{
}

bool PozyxModule::dataReady()
{
    if (!ips_settings_updated || !udp_socket->is_open())
    {
        return false;
    }

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - lastPosFrameTS;

    // check max time
    if (currentFrame >= 1 && udp_socket->available() == 0)
    {
        std::chrono::seconds elapsed_secs = std::chrono::duration_cast<std::chrono::seconds>(elapsed);
        double secs = elapsed_secs.count();
        if (secs > max_secs_for_data)
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_POSITIONING_STOPPED);
            Log::critical("PozyxModule: Pozyx-server has STOPPED publishing IPS frames at UDP port for more than {} seconds. Stopping PozyxModule...", max_secs_for_data);

            stopDevice();
            deviceOn = false;
        }

        return false;
    }

    lastPosFramePeriodms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

    // update
    lastPosFrameTS = now;

    // udp_remote_endpoint will be populated by receive_from()
    nBytesReceivedFromPositioning = udp_socket->receive_from(boost::asio::buffer(recv_buf),
                                                             udp_remote_endpoint, 0, udp_error);

    if (udp_error && udp_error != boost::asio::error::message_size)
    {
        Log::error("PozyxModule: Error in dataReady. Code: {}", boost::system::system_error(udp_error).what());
        return false;
    }

    return true;
}

bool PozyxModule::quitDevice()
{
    /*
    PozyxResponses::ShortCommandResp cmdresp;
    messageProcessor.requestStop(cmdresp);
    if(!cmdresp.success)
    {
        // it is commonly 'already stopped'
        Log::error("PozyxModule: pozyx-server failed stop. Error: {}", cmdresp.error);
    }
    */

    /*
    // TODO: send kill command when pozyx-server does not crash when receiving kill command
    PozyxResponses::ShortCommandResp cmdresp;
    messageProcessor.requestKill(cmdresp);
    if(!cmdresp.success)
    {
        // it is commonly 'already stopped'
        Log::error("PozyxModule: pozyx-server failed kill. Error: {}", cmdresp.error);
    }
    */

    if (udp_socket->is_open())
    {
        udp_socket->close();
    }

    Log::info("PozyxModule: quitting");
    return true;
}

bool PozyxModule::sendMessage()
{
    // Get only valid data (not the whole buffer)
    std::string frame_str(recv_buf.c_array(), nBytesReceivedFromPositioning);

    PozyxResponses::PositioningFrame frame;
    messageProcessor.extractPositioningFrame(frame_str, frame);

    if (!frame.success)
    {
        Log::error("PozyxModule: invalid frame {}. Not sending it", currentFrame);
        return false;
    }

    PositioningFrame posData;

    posData.timestamp = frame.timestamp;
    posData.x = frame.coordinates.x;
    posData.y = frame.coordinates.y;
    posData.z = frame.coordinates.z;

    // pozyxserver is not providing pitch and roll (ignoring those incoming values)
    posData.roll = 0.0;
    posData.pitch = 0.0;
    posData.yaw = frame.yaw;

    Message msg;
    msg.add_raw(&posData, sizeof(PositioningFrame));
    pubsocket->send(msg);

    Log::info("PozyxModule: totalFramesSent {} RPY {} {} {} XYZ {} {} {}, IPS-frame period from pozyxserver {} ms",
              currentFrame,
              posData.roll, posData.pitch, posData.yaw,
              posData.x, posData.y, posData.z, lastPosFramePeriodms.count());

    // update
    currentFrame++;
    nBytesReceivedFromPositioning = 0;

    return true;
}

void PozyxModule::onMessage(const Message &m)
{
    std::stringstream ss;

    uint8_t module = airt::getMessageAction(m);
    switch (module)
    {
    case StdMessage::IPS_SYSTEM:
        Log::info("PozyxModule: received command IPS_SYSTEM");

        if (messageProcessor.requestSystem(resp_system))
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LOCALTAG_CONNECTED);

            Log::info("PozyxModule: got system response: [status {}, rss {}, id {}, hexId {}, distance {}]",
                      resp_system.status,
                      resp_system.device.rss, resp_system.device.id,
                      resp_system.device.hexId, resp_system.device.distance);
        }
        else
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
        }
        break;

    case StdMessage::IPS_DISCOVER:
        Log::info("PozyxModule: received command IPS_DISCOVER");

        if (messageProcessor.requestDiscoverFull(resp_discover))
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_DISCOVERED);

            ss.clear();
            ss << "Tags found: " << resp_discover.tags.size() << ", ids [";
            for (size_t i = 0; i < resp_discover.tags.size(); i++)
            {
                ss << resp_discover.tags[i].id << ", ";
            }
            ss << "]";

            ss << ", Anchors found: " << resp_discover.anchors.size() << ", ids [";
            for (size_t i = 0; i < resp_discover.anchors.size(); i++)
            {
                ss << resp_discover.anchors[i].id << ", ";
            }
            ss << "]";

            Log::info("PozyxModule: got discover response: {}", ss.str());

            sendAnchorsIDsList(resp_discover);

            if (resp_discover.anchors.size() >= 8)
            {
                if (settings.anchors.size() == 0)
                {
                    if (!messageProcessor.loadSettings(pozyx_settingsfile_path, settings))
                    {
                        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
                        break;
                    }
                }

                // send previous session anchors-locations (the ids might not be the same)
                sendAnchorsLocations(settings);
            }
        }
        else
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
        }
        break;

    case StdMessage::IPS_ANCHOR_MANUAL_CONFIG:
    {
        Log::info("PozyxModule: received IPS_ANCHOR_MANUAL_CONFIG");
        const PositioningAnchorManualConfigHdr *hdr = m.get<const PositioningAnchorManualConfigHdr *>(0);

        if (hdr->numAnchors < 8)
        {
            Log::error("PozyxModule: not enough anchors-manual-config received from client: {}", hdr->numAnchors);
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
            break;
        }
        else
        {
            for (size_t i = 1; i < m.parts(); i++)
            {
                const PositioningAnchorManualConfig *a = m.get<const PositioningAnchorManualConfig *>(i);
                Log::info("PozyxModule: received anchor-manual-config: id {} order {} x {} y {} z {}",
                          a->id, a->order, a->x, a->y, a->z);

                messageProcessor.setAnchorConfig(a->id, a->order, a->x, a->y, a->z, settings, resp_discover.anchors);
            }

            if (!messageProcessor.checkSettings(settings))
            {
                Log::error("PozyxModule: failed when checking settings after setAnchorConfig");
                airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
                Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
                break;
            }

            // backup
            messageProcessor.saveSettings(pozyx_settingsfile_path + "." + airt::getTimeStamp() + ".bak", settings);
            // override file
            messageProcessor.saveSettings(pozyx_settingsfile_path, settings);

            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_ANCHORS_MANUAL_CONFIG_ACCEPTED);
            Log::info("PozyxModule: sent IPS_ANCHORS_MANUAL_CONFIG_ACCEPTED");
        }
    }
    break;

    case StdMessage::IPS_ANCHOR_TOBE_AUTOCALIBRATED:
    {
        Log::info("PozyxModule: received IPS_ANCHOR_TOBE_AUTOCALIBRATED");
        const PositioningAnchorTobeAutocalibratedHdr *hdr = m.get<const PositioningAnchorTobeAutocalibratedHdr *>(0);

        if (hdr->numAnchors < 8)
        {
            Log::error("PozyxModule: not enough anchors-tobe-autocalibrated received from client: {}", hdr->numAnchors);
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
            break;
        }
        else
        {
            std::vector<PozyxPayloads::AutocalibAnchorConfig> anchors_tobe_autocalibrated;
            for (size_t i = 1; i < m.parts(); i++)
            {
                const PositioningAnchorTobeAutocalibrated *a = m.get<const PositioningAnchorTobeAutocalibrated *>(i);
                Log::info("PozyxModule: received anchor-tobe-autocalibrated: id {} order {} x {} y {} z {}",
                          a->id, a->order, a->x, a->y, a->z);

                PozyxPayloads::AutocalibAnchorConfig aac;
                aac.id = std::to_string(a->id);
                aac.order = a->order;
                aac.coordinates.x = a->x;
                aac.coordinates.y = a->y;
                aac.coordinates.z = a->z;

                anchors_tobe_autocalibrated.push_back(aac);
            }

            messageProcessor.buildAutocalib(anchors_tobe_autocalibrated, autocalib);
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_ANCHORS_TOBE_AUTOCALIBRATED_ACCEPTED);
            Log::info("PozyxModule: sent IPS_ANCHORS_TOBE_AUTOCALIBRATED_ACCEPTED");
        }
    }
    break;

    case StdMessage::IPS_AUTOCALIBRATE:
        Log::info("PozyxModule: received command: IPS_AUTOCALIBRATE");

        if (autocalib.anchors.size() == 0)
        {
            Log::error("PozyxModule: autocalibration payload does not have any anchor");
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
            break;
        }

        if (!messageProcessor.requestAutocalibrate(autocalib, resp_autocalibrate))
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
            break;
        }
        else
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_AUTOCALIBRATED);

            Log::info("PozyxModule: got autocalibrate response: succes {} score {} connectivity {}",
                      resp_autocalibrate.success, resp_autocalibrate.score, resp_autocalibrate.connectivity);

            sendAnchorsLocations(resp_autocalibrate.anchors);
        }

        break;

    case StdMessage::IPS_UPDATE_SETTINGS:
    {
        Log::info("PozyxModule: received command IPS_UPDATE_SETTINGS");

        /*
        if(currentFrame > 0)
        {
            Log::warn("PozyxModule: already receiving positioning frames, total {}, ignoring IPS_UPDATE_SETTINGS...", currentFrame);
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_UPDATED_SETTINGS);
            break;
        }
        */

        if (settings.anchors.size() == 0)
        {
            if (!messageProcessor.loadSettings(pozyx_settingsfile_path, settings))
            {
                airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
                Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
                break;
            }
        }

        if (!messageProcessor.requestUpdateSettings(settings, resp_updatesettings))
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
            break;
        }

        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_UPDATED_SETTINGS);
        ips_settings_updated = true;

        sendAnchorsLocations(settings);

        // devices have changed to target channel, keep it for next session
        messageProcessor.setAnchorsAndTagsToTargetUwb(settings.uwb, settings);
        // backup
        messageProcessor.saveSettings(pozyx_settingsfile_path + "." + airt::getTimeStamp() + ".bak", settings);
        // override
        messageProcessor.saveSettings(pozyx_settingsfile_path, settings);

        ss.clear();
        ss << "success: " << (resp_updatesettings.success ? "true " : "false ");
        ss << ", tags [";

        for (size_t i = 0; i < resp_updatesettings.tags_updated.size(); i++)
        {
            ss << " {" << resp_updatesettings.tags_updated[i].id << ", " << std::boolalpha << resp_updatesettings.tags_updated[i].success << "},";
        }
        ss << "]";

        Log::info("PozyxModule: got updatesettings response: {}", ss.str());

        break;
    }
    case StdMessage::IPS_START_POSITIONING:
    {
        Log::info("PozyxModule: received command IPS_START_POSITIONING");

        if (!ips_settings_updated)
        {
            Log::error("PozyxModule: pozyx-settings have not been updated yet");
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
            break;
        }

        /*
        if(currentFrame > 0)
        {
            Log::warn("PozyxModule: already receiving positioning frames, total {}, ignoring IPS_START_POSITIONING...", currentFrame);
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_POSITIONING_STARTED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
            break;
        }
        */

        PozyxResponses::ShortCommandResp cmdresp;
        messageProcessor.requestStart(cmdresp);

        if (!cmdresp.success)
        {
            // it is commonly 'Already positioning'
            //            Log::error("PozyxModule: pozyx-server failed start. Error: {}", cmdresp.error);
        }

        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_POSITIONING_STARTED);
        Log::info("PozyxModule: sent IPS_POSITIONING_STARTED");

        // open the tap and start enqueing in the o.s queue
        if (!udp_socket->is_open())
        {
            udp_socket->open(udp::v4());
            udp_socket->bind(udp::endpoint(udp::v4(), pozyx_udpport));
        }

        // reset
        nBytesReceivedFromPositioning = 0;
        currentFrame = 0;
        lastPosFrameTS = std::chrono::steady_clock::now();

        Log::info("PozyxModule: starting... Current frame: {}", currentFrame);
    }
    break;
    case StdMessage::IPS_STOP_POSITIONING:
    {
        Log::info("PozyxModule: received command IPS_STOP_POSITIONING");

        // only stop receiving (not actually sending a stop request to pozyx-server)
        // close the tap to avoid the o.s. enqueues unrequired data
        if (udp_socket->is_open())
        {
            udp_socket->close();
        }

        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_POSITIONING_STOPPED);

        Log::info("PozyxModule: stopping... Current frame: {}", currentFrame);
    }
    break;
    case StdMessage::IPS_DISCOVER_DRONETAGS:
        Log::info("PozyxModule: received command IPS_DISCOVER_DRONETAGS");

        if (!messageProcessor.requestDiscoverFull(resp_discover)) // returns only non-local (non-connected to up2) tags
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
            break;
        }
        else
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_DISCOVERED);

            ss.clear();
            ss << "Tags found: " << resp_discover.tags.size() << ", ids [";
            for (size_t i = 0; i < resp_discover.tags.size(); i++)
            {
                ss << resp_discover.tags[i].id << ", ";
            }
            ss << "]";

            Log::info("PozyxModule: got discover response: {}", ss.str());

            sendTagsIDsList(resp_discover.tags);

            if (resp_discover.tags.size() >= 4) // (sw, nw, ne, se)
            {
                if (settings.tags.size() == 0)
                {
                    if (!messageProcessor.loadSettings(pozyx_settingsfile_path, settings))
                    {
                        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
                        break;
                    }
                }

                // send previous session drone-tags (the ids might not be the same as the discovered)
                sendDroneTags(settings);
            }
        }

        break;

    case StdMessage::IPS_DRONETAGS_MANUAL_CONFIG:
    {
        const PositioningDroneTagsManual *dtm = m.get<const PositioningDroneTagsManual *>(0);

        Log::info("PozyxModule: received IPS_DRONETAGS_MANUAL_CONFIG: droneWidthmm {} droneHeightmm {} sw {} nw {} ne {} se {}",
                  dtm->droneWidthmm, dtm->droneHeightmm, dtm->swID, dtm->nwID, dtm->neID, dtm->seID);

        if (settings.tags.size() == 0)
        {
            if (!messageProcessor.loadSettings(pozyx_settingsfile_path, settings))
            {
                airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
                Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
                break;
            }
        }

        messageProcessor.setDroneTagsConfig(dtm->droneWidthmm, dtm->droneHeightmm, dtm->swID, dtm->nwID, dtm->neID, dtm->seID, settings, resp_discover.tags);

        // backup
        messageProcessor.saveSettings(pozyx_settingsfile_path + "." + airt::getTimeStamp() + ".bak", settings);
        // override file
        messageProcessor.saveSettings(pozyx_settingsfile_path, settings);

        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_DRONETAGS_MANUAL_CONFIG_ACCEPTED);
        Log::info("PozyxModule: sent IPS_DRONETAGS_MANUAL_CONFIG_ACCEPTED");
    }
    break;
    case StdMessage::IPS_CLEAR_POZYXSETTINGS:
    {
        Log::info("PozyxModule: received command IPS_CLEAR_POZYXSETTINGS");

        // if no loaded, load from file
        if (settings.tags.size() == 0)
        {
            if (!messageProcessor.loadSettings(pozyx_settingsfile_path, settings))
            {
                airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
                Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
                break;
            }
        }

        messageProcessor.clearSettings(settings);
        messageProcessor.saveSettings(pozyx_settingsfile_path, settings);

        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_POZYXSETTINGS_CLEARED);
        Log::info("PozyxModule: pozyx-settings have been cleared and save to file");
    }
    break;
    case StdMessage::IPS_GET_DRONEFILTER:
    {
        Log::info("PozyxModule: received command IPS_GET_DRONEFILTER");

        // if no loaded, load from file
        if (settings.tags.size() == 0)
        {
            if (!messageProcessor.loadSettings(pozyx_settingsfile_path, settings))
            {
                airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
                Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
                break;
            }
        }

        Message msg;

        PositioningGetDroneFilterNotification gdf;
        gdf.updatePeriod = settings.drone.options_filter_updatePeriod;
        gdf.movementFreedom = settings.drone.options_filter_movementFreedom;
        msg.add_raw(&gdf, sizeof(gdf));

        pubsocket->send(msg);

        Log::info("PozyxModule: sent DroneFilter: updatePeriod {} movementFreedom {} from settings",
                  settings.drone.options_filter_updatePeriod, settings.drone.options_filter_movementFreedom);
    }
    break;

    case StdMessage::IPS_SET_DRONEFILTER:
    {
        const PositioningSetDroneFilter *sdf = m.get<const PositioningSetDroneFilter *>(0);

        Log::info("PozyxModule: received IPS_SET_DRONEFILTER: updatePeriod {} movementFreedom {}",
                  sdf->updatePeriod, sdf->movementFreedom);

        // if no loaded, load from file
        if (settings.tags.size() == 0)
        {
            if (!messageProcessor.loadSettings(pozyx_settingsfile_path, settings))
            {
                airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
                Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
                break;
            }
        }

        // updatePeriod: High update rate means a lower value
        // movementFreedom: default value approximates a medium strength filter
        if (!messageProcessor.setDroneFilter(sdf->updatePeriod, sdf->movementFreedom, settings))
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
            break;
        }

        messageProcessor.saveSettings(pozyx_settingsfile_path, settings);

        airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_SET_DRONEFILTER_NOTIFICATION);
        Log::info("PozyxModule: sent IPS_SET_DRONEFILTER_NOTIFICATION");
    }
    break;
    case StdMessage::IPS_GET_ANCHORS_FROM_FILE:
    {
        Log::info("PozyxModule: received IPS_GET_ANCHORS_FROM_FILE");

        if (!messageProcessor.loadSettings(pozyx_settingsfile_path, settings))
        {
            airt::sendNotification(*pubsocket, StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_LAST_REQUEST_HAS_FAILED);
            Log::error("PozyxModule: sent IPS_LAST_REQUEST_HAS_FAILED");
            break;
        }
        sendAnchorsLocations(settings);

        Log::info("PozyxModule: sent anchors-locations from {}", pozyx_settingsfile_path);
    }
    break;

    default:
        Log::error("PozyxModule: Unknown command: {}", airt::to_string(m));
        break;
    }

    return;
}

bool PozyxModule::sendAnchorsIDsList(const PozyxResponses::Discover &discover)
{
    Message msg;
    positioningAnchorsListHdr listHdr;
    std::vector<positioningAnchorsListItem> items;

    listHdr.numAnchors = discover.anchors.size();
    msg.add_raw(&listHdr, sizeof(listHdr));

    for (size_t i = 0; i < discover.anchors.size(); i++)
    {
        positioningAnchorsListItem item;
        item.id = std::stoi(discover.anchors[i].id);
        items.push_back(item);
    }

    msg.add_raw(&items[0], sizeof(positioningAnchorsListItem) * items.size());

    pubsocket->send(msg);
    Log::info("PozyxModule: sending anchors IDs list. Num anchors: {}", items.size());

    return true;
}

bool PozyxModule::sendAnchorsLocations(const std::vector<PozyxResponses::AutocalibratedAnchor> &anchors)
{
    Message msg;
    positioningAnchorLocationHeader locHdr;
    locHdr.numAnchors = anchors.size();
    msg.add_raw(&locHdr, sizeof(locHdr));

    for (size_t i = 0; i < anchors.size(); i++)
    {
        positioningAnchorLocation loc;

        loc.id = std::atoi(anchors[i].id.c_str());
        loc.order = anchors[i].order;
        loc.x = anchors[i].coordinates.x;
        loc.y = anchors[i].coordinates.y;
        loc.z = anchors[i].coordinates.z;

        msg.add_raw(&loc, sizeof(loc));

        Log::info("PozyxModule: sending anchor location from autocalibrate: id {} order {} x {} y {} z {}",
                  loc.id, loc.order, loc.x, loc.y, loc.z);
    }

    pubsocket->send(msg);

    return true;
}

bool PozyxModule::sendAnchorsLocations(const PozyxPayloads::Settings &settings)
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

bool PozyxModule::sendTagsIDsList(const std::vector<PozyxPayloads::Tag> &tags)
{
    Message msg;
    PositioningDroneTagsListHdr listHdr;
    std::vector<PositioningDroneTagsListItem> items;

    for (size_t i = 0; i < tags.size(); i++)
    {
        if (!tags[i].isLocal)
        {
            PositioningDroneTagsListItem item;
            item.id = std::stoi(tags[i].id);
            items.push_back(item);
        }
    }

    listHdr.numTags = items.size();

    msg.add_raw(&listHdr, sizeof(listHdr));
    msg.add_raw(&items[0], sizeof(PositioningDroneTagsListItem) * items.size());

    pubsocket->send(msg);
    Log::info("PozyxModule: sending drone-tags IDs list. Num drone-tags (excluding local-tag): {}", items.size());

    return true;
}

bool PozyxModule::sendDroneTags(const PozyxPayloads::Settings &settings)
{
    // checking
    int ntags_ok = 0;
    for (size_t i = 0; i < settings.tags.size(); i++)
    {
        if (settings.tags[i].id == settings.drone.tag_sw ||
            settings.tags[i].id == settings.drone.tag_nw ||
            settings.tags[i].id == settings.drone.tag_ne ||
            settings.tags[i].id == settings.drone.tag_se)
        {
            ntags_ok++;
        }
    }

    if (ntags_ok != 4)
    {
        Log::critical("PozyxModule: sendDroneTags: settings are malformed, drone-tags (sw,nw,ne,se) ids are not the same as ids in tags[] array");
        return false;
    }

    // sending message
    Message msg;

    PositioningDroneTags dt;
    dt.droneWidthmm = settings.drone.droneWidthmm;
    dt.droneHeightmm = settings.drone.droneHeightmm;
    dt.swID = std::stoi(settings.drone.tag_sw);
    dt.nwID = std::stoi(settings.drone.tag_nw);
    dt.neID = std::stoi(settings.drone.tag_ne);
    dt.seID = std::stoi(settings.drone.tag_se);

    msg.add_raw(&dt, sizeof(dt));

    pubsocket->send(msg);
    Log::info("PozyxModule: sending dronetags from settings: droneWidthmm {} droneHeightmm {} sw {} nw {} ne {} se {}",
              dt.droneWidthmm, dt.droneHeightmm, dt.swID, dt.nwID, dt.neID, dt.seID);

    return true;
}
