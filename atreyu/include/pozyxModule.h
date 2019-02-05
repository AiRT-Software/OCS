#pragma once

#include <stdMessage.h>
#include "module.h"
#include <pozyxMessageProcessor.h>
#include <pozyxResponses.h>


// for using udp socket
#include <boost/array.hpp>
#include <boost/asio.hpp>
#define UPD_BUFFER_SIZE 2048

#include <iomanip>
#include <vector>


namespace airt
{
class Context;

class PozyxModule : public Module
{
  public:

#include "interface/pozyxmodule.h"

    PozyxModule(const std::string &cmdportname, const std::string &subportname,
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

    /**
     * @brief PozyxModule::onMessage receives non-standard command messages from the atreyu server addressed to this specific module.
     * Depending on the command Message an apropiate http request will be sent to the pozyx server.
     * @param m is the input command Message from atreyu (commonly: SYSTEM, DISCOVER, AUTOCALIBRATE, UPDATE_SETTINGS).
     * The standard START and STOP commands will start/stop the stream from the positioning udp port.
     */
    void onMessage(const Message &m) override;

  private:
    PozyxModule(const PozyxModule &other) = delete;
    PozyxModule &operator=(PozyxModule other) = delete;

    bool ips_settings_updated;

    /**
     * @brief udp_socket is a boost udp socket for getting continously positioning stream from the poxyzserver.
     * The udp socket will be connected when a START command is received, and it will be closed when a STOP command is received.
     */
    std::unique_ptr<boost::asio::ip::udp::socket> udp_socket;
    boost::asio::io_service udp_io_service;
    boost::asio::ip::udp::endpoint udp_remote_endpoint;
    boost::system::error_code udp_error;
    boost::array<char, UPD_BUFFER_SIZE> recv_buf;

    int pozyx_udpport;
    float max_secs_for_data;
    size_t currentFrame;

    /**
     * @brief lastPosFrameTS last local time_point at which a positioning frame was received.
     */
    std::chrono::time_point<std::chrono::steady_clock> lastPosFrameTS;
    std::chrono::milliseconds lastPosFramePeriodms;

    /**
     * @brief nBytesReceivedFromPositioning is the valid number of bytes received from the pozyx-server
     * It should be less than UPD_BUFFER_SIZE.
     */
    size_t nBytesReceivedFromPositioning;

    /**
     * @brief pozyx_settingsfile_path is the pozyx-server settings file used for loading/saving
     * the positioning settings (channel, anchors ids, tags ids, etc)
     */
    std::string pozyx_settingsfile_path;

    PozyxMessageProcessor messageProcessor;
    PozyxResponses::System resp_system;
    PozyxResponses::Discover resp_discover;
    PozyxResponses::Autocalibrate resp_autocalibrate;
    PozyxPayloads::Autocalib autocalib;
    PozyxResponses::UpdateSettings resp_updatesettings;
    PozyxPayloads::Settings settings;

    bool sendAnchorsIDsList(const PozyxResponses::Discover & resp_discover);
    bool sendAnchorsLocations(const PozyxResponses::Autocalibrate & resp_autocalibrate);
    bool sendAnchorsLocations(const PozyxPayloads::Settings & settings);
    bool sendAnchorsLocations(const std::vector<PozyxResponses::AutocalibratedAnchor> & anchors);

    bool sendTagsIDsList(const std::vector<PozyxPayloads::Tag> & tags);
    bool sendDroneTags(const PozyxPayloads::Settings & settings);

    int mappingCamDistancemm;
};
};
