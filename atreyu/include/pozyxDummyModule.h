#pragma once

#include <stdMessage.h>
#include "module.h"
#include <pozyxMessageProcessor.h>
#include <pozyxResponses.h>

#include <glm/vec3.hpp>
#include <iomanip>

namespace airt
{
class Context;

class PozyxDummyModule : public Module
{
  public:

#include "interface/pozyxmodule.h"

    PozyxDummyModule(const std::string &cmdportname, const std::string &pubportname, const std::string &subportname,
                std::shared_ptr<Context> context = std::shared_ptr<Context>());

    /**
     * @brief PozyxModule::dataReady gets data from the o.s. udp queue. It retrieves the data to a local buffer.
     * It remains attending from any remote_endpoint (like the pozyxserver) at positioning port.
     * 'udp_remote_endpoint' value will be populated by receive_from().
     * When pozyx-server stopped providing positioning frames for more than max_secs_for_data,
     * then a STOPPED message is sent and the PozyxModule stops.
     * @return false when no data has been received, true when some data has been received.
     */
    bool dataReady() override;
    bool sendMessage() override;
    bool startDevice() override;
    void stopDevice() override;
    bool quitDevice() override;
    void configureSubscriptions(const std::string &mainpublisher) override;
    void onNotification(const Message &m) override;

    /**
     * @brief PozyxModule::onMessage receives non-standard command messages from the atreyu server addressed to this specific module.
     * Depending on the command Message an apropiate http request will be sent to the pozyx server.
     * @param m is the input command Message from atreyu (commonly: SYSTEM, DISCOVER, AUTOCALIBRATE, UPDATE_SETTINGS).
     * The standard START and STOP commands will start/stop the stream from the positioning udp port.
     */
    void onMessage(const Message &m) override;

  private:
    PozyxDummyModule(const PozyxDummyModule &other) = delete;
    PozyxDummyModule &operator=(PozyxDummyModule other) = delete;

    std::vector<glm::vec3> waypoints;
    size_t nextWaypoint;
    glm::vec3 currentPos;
    float speedMeterPerSec;
    bool moving;
    float publishPeriodS; // period between position updates (s)
    glm::vec3 errorStdDev; // standard deviation of errors in the position
    size_t currentFrame;

    // for simulating pozyx requests and responses
    std::string pozyx_settingsfile_path;
    PozyxMessageProcessor messageProcessor;
    PozyxPayloads::Settings settings;
    PozyxPayloads::Autocalib autocalib;
    PozyxResponses::Autocalibrate resp_autocalibrate;
    PozyxResponses::Discover resp_discover;
    bool ips_settings_updated;

    bool sendAnchorsIDsList(const PozyxPayloads::Settings & settings);
    bool sendAnchorsLocations(const PozyxPayloads::Settings & settings);
    bool sendAnchorsLocations(const PozyxResponses::Autocalibrate &autocalibrate);

    std::vector<PositioningAnchorManualConfig> anchors_manual;
    std::vector<PozyxPayloads::AutocalibAnchorConfig> anchors_tobe_autocalibrated;
};

};
