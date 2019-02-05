#pragma once
#include <rest.h>
#include <vector>

namespace PozyxResponses {
    class System;
    class Discover;
    class Autocalibrate;
    class UpdateSettings;
    class PositioningFrame;
    class ShortCommandResp;
}

namespace PozyxPayloads {
    class Positioning;
    class Drone;
    class Settings;
    class Autocalib;
    class AutocalibAnchorConfig;
    class Uwb;
    class Tag;
    class Anchor;
}


namespace airt {

class PozyxMessageProcessor
{
public:
    PozyxMessageProcessor(std::string ip = "10.42.0.100", std::string httpport = "5000");
    ~PozyxMessageProcessor();

    void setEndpoints(const std::string & ip, const std::string & httpport);

    bool requestSystem(PozyxResponses::System & resp_system);
    bool requestDiscover(PozyxResponses::Discover & resp_discover);
    bool requestDiscoverFull(PozyxResponses::Discover & resp_discover);
    bool requestAutocalibrate(const PozyxPayloads::Autocalib & autocalib, PozyxResponses::Autocalibrate & resp_autocalibrate);
    bool requestUpdateSettings(const PozyxPayloads::Settings & settings, PozyxResponses::UpdateSettings & resp_updatesettings);

    bool requestShortCommand(const std::string & endpoint, PozyxResponses::ShortCommandResp & cmdresp);
    bool requestStart(PozyxResponses::ShortCommandResp & start);
    bool requestStop(PozyxResponses::ShortCommandResp & stop);
    bool requestKill(PozyxResponses::ShortCommandResp & kill);

    bool extractPositioningFrame(const std::string & ipsframe_json, PozyxResponses::PositioningFrame & ipsframe);

    bool loadSettings(std::string file_path, PozyxPayloads::Settings & settings);
    bool saveSettings(std::string file_path, const PozyxPayloads::Settings & settings);
    bool checkSettings(const PozyxPayloads::Settings & settings);
    PozyxPayloads::Settings buildSettings(const PozyxPayloads::Positioning & positioning, const PozyxPayloads::Drone & drone,
                                          const PozyxPayloads::Uwb & uwb_target, const PozyxResponses::Discover & discovered);

    std::string settingsToString(const PozyxPayloads::Settings & settings, bool printToLog = false);
    std::string autocalibToString(const PozyxPayloads::Autocalib & autocalib);

    bool buildAutocalib(const PozyxPayloads::Settings & settings, PozyxPayloads::Autocalib & autocalib);
    bool buildAutocalib(const std::vector<PozyxPayloads::AutocalibAnchorConfig> & autocalib_anchors, PozyxPayloads::Autocalib & autocalib);
    bool setAnchorConfig(int id, uint8_t order, int x, int y, int z, PozyxPayloads::Settings & settings, const std::vector<PozyxPayloads::Anchor> & discovered_anchors);
    bool getSyntheticAutocalibrateResponse(const PozyxPayloads::Autocalib & autocalib, PozyxResponses::Autocalibrate & autocalibrate);
    bool setDroneTagsConfig(int drone_width_mm, int drone_height_mm, int sw_id, int nw_id, int ne_id, int se_id, PozyxPayloads::Settings &settings, const std::vector<PozyxPayloads::Tag> & discovered_tags);
    bool setDroneFilter(float updatePeriod_secs, int movementFreedom, PozyxPayloads::Settings & settings);
    bool clearSettings(PozyxPayloads::Settings & settings);
    bool setAnchorsAndTagsToTargetUwb(const PozyxPayloads::Uwb & target_uwb, PozyxPayloads::Settings & settings);
private:
    Rest rest;
    std::string ip;
    std::string httpport;

    std::string endpoint_system;
    std::string endpoint_discover;
    std::string endpoint_discoverfull;
    std::string endpoint_autocalibrate;
    std::string endpoint_updatesettings;
    std::string endpoint_start;
    std::string endpoint_stop;
    std::string endpoint_kill;
};

}
