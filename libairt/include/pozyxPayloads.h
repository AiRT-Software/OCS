#pragma once
#include <iostream>
#include <vector>


namespace PozyxPayloads
{
    class Int3
    {
    public:
        int x, y, z;

        bool x_ToBeAutocalibrated;
        bool y_ToBeAutocalibrated;
        bool z_ToBeAutocalibrated;


        Int3(int x_ = 0, int y_ = 0, int z_ = 0,
             bool x_ToBeAutocalibrated_ = false,
             bool y_ToBeAutocalibrated_ = false,
             bool z_ToBeAutocalibrated_ = false
             )
            : x(x_), y(y_), z(z_),
              x_ToBeAutocalibrated(x_ToBeAutocalibrated_),
              y_ToBeAutocalibrated(y_ToBeAutocalibrated_),
              z_ToBeAutocalibrated(z_ToBeAutocalibrated_)
        {}

        Int3(const Int3 &) = default;
        Int3(Int3 &) = default;

        Int3 & operator =(const Int3 & other)
        {
            x = other.x;
            y = other.y;
            z = other.z;

            x_ToBeAutocalibrated = other.x_ToBeAutocalibrated;
            y_ToBeAutocalibrated = other.y_ToBeAutocalibrated;
            z_ToBeAutocalibrated = other.z_ToBeAutocalibrated;

            return * this;
        }
    };

    class Float3
    {
    public:
        float x, y, z;
        Float3(float x_ = 0.0, float y_ = 0.0, float z_ = 0.0)
            : x(x_), y(y_), z(z_)
        {}

        Float3(const Float3 &) = default;
        Float3(Float3 &) = default;

        Float3 & operator =(const Float3 & other)
        {
            x = other.x;
            y = other.y;
            z = other.z;

            return * this;
        }
    };

    // Basic settings payloads structures
    class Uwb
    {
    public:
        int channel;
        std::string bitrate;
        std::string prf;
        int plen;
        float gain;

        Uwb(int channel_= 0, std::string bitrate_ = "",
            std::string prf_ = "", int plen_ = 0, float gain_= 0.0)
            : channel(channel_), bitrate{std::move(bitrate_)},
              prf{std::move(prf_)}, plen(plen_), gain(gain_)
        {}

        Uwb(const Uwb &) = default;
        Uwb(Uwb &&) = default;

        Uwb & operator =(const Uwb & other)
        {
            channel = other.channel;
            bitrate = other.bitrate;
            prf = other.prf;
            plen = other.plen;
            gain = other.gain;

            return *this;
        }
    };

    class Device
    {
    public:
        int rss;
        std::string hexId;
        std::string fwVersion;
        std::string hwVersion;
        int distance;

        std::string id;
        Uwb settings_uwb;
        bool inUse;

        Device(int rss_ = 0,  std::string hexId_ = "", std::string fwVersion_= "",
               std::string hwVersion_ = "", int distance_ = -1,
               std::string id_ = "", Uwb settings_uwb_ = Uwb(), bool inUse_ = true)
            :
              rss(rss_), hexId{std::move(hexId_)}, fwVersion{std::move(fwVersion_)},
              hwVersion{std::move(hwVersion_)}, distance(distance_),
              id{std::move(id_)}, settings_uwb(settings_uwb_), inUse(inUse_)
        {}

        Device(const Device &) = default;
        Device(Device &&) = default;

        Device & operator =(const Device & other)
        {
            rss = other.rss;
            hexId = other.hexId;
            fwVersion = other.fwVersion;
            hwVersion = other.hwVersion;
            distance = other.distance;
            id = other.id;
            settings_uwb = other.settings_uwb;
            inUse = other.inUse;

            return *this;
        }
    };

    class Tag : public Device
    {
    public:
        bool isLocal;

        explicit Tag(bool isLocal_ = false)
            :
              isLocal(isLocal_)
        {}

        Tag(const Tag &) = default;
        Tag(Tag &&) = default;

        Tag & operator =(const Tag & other)
        {
            isLocal = other.isLocal;

            rss = other.rss;
            hexId = other.hexId;
            fwVersion = other.fwVersion;
            hwVersion = other.hwVersion;
            distance = other.distance;
            id = other.id;
            settings_uwb = other.settings_uwb;
            inUse = other.inUse;

            return *this;
        }
    };

    class AnchorCoordinates
    {
    public:
        int x, y, z;

        AnchorCoordinates(int x_ = 0, int y_ = 0, int z_ = 0)
            : x(x_), y(y_), z(z_)
        {}

        AnchorCoordinates(const AnchorCoordinates &) = default;
        AnchorCoordinates(AnchorCoordinates &&) = default;

        AnchorCoordinates & operator =(const AnchorCoordinates & other)
        {
            x = other.x;
            y = other.y;
            z = other.z;

            return *this;
        }
    };

    class Anchor : public Device
    {
    public:
        AnchorCoordinates coordinates;

        explicit Anchor(AnchorCoordinates coordinates_ = AnchorCoordinates())
            :
              coordinates(coordinates_)
        {}

        Anchor(const Anchor &) = default;
        Anchor(Anchor &&) = default;

        Anchor & operator =(const Anchor & other)
        {
            coordinates = other.coordinates;

            rss = other.rss;
            hexId = other.hexId;
            fwVersion = other.fwVersion;
            hwVersion = other.hwVersion;
            distance = other.distance;
            id = other.id;
            settings_uwb = other.settings_uwb;
            inUse = other.inUse;

            return *this;
        }
    };

    // settings.positioning
    class Positioning
    {
    public:
        bool enabled;
        std::string algorithm;
        std::string dimension;
        int height;
        std::string rangingProtocol;
        std::string filterType;
        int filterStrength;

        Positioning(bool enabled_ = true, std::string algorithm_ = "UWB_ONLY", std::string dimension_ = "3D",
                    int height_ = 0, std::string rangingProtocol_ = "FAST", std::string filterType_ = "NONE",
                    int filterStrength_ = 0)
            : enabled(enabled_), algorithm{std::move(algorithm_)}, dimension{std::move(dimension_)},
              height(height_), rangingProtocol{std::move(rangingProtocol_)}, filterType{std::move(filterType_)},
              filterStrength(filterStrength_)
        {}

        Positioning(const Positioning &) = default;
        Positioning(Positioning &&) = default;

        Positioning & operator =(const Positioning & other)
        {
            enabled = other.enabled;
            algorithm = other.algorithm;
            dimension = other.dimension;
            height = other.height;
            rangingProtocol = other.rangingProtocol;
            filterType = other.filterType;
            filterStrength = other.filterStrength;

            return *this;
        }
    };

    // settings.drone
    class Drone
    {
    public:
        int droneWidthmm;
        int droneHeightmm;
        std::string options_mode;
        std::string options_onFail;
        int options_orientationDimension;
        float options_filter_updatePeriod;
        int options_filter_movementFreedom;
        std::string tag_sw;
        std::string tag_nw;
        std::string tag_ne;
        std::string tag_se;

        Drone(int droneWidthmm_ = 0, int droneHeightmm_ = 0,
              std::string options_mode_ = "individual", std::string options_onFail_ = "individual",
              int options_orientationDimension_ = 0,
              float options_filter_updatePeriod_ = 0.0, int options_filter_movementFreedom_ = 0,
              std::string tag_sw_ = "0x0000", std::string tag_nw_ = "0x1111",
              std::string tag_ne_ = "0x2222", std::string tag_se_ = "0x3333"
              )
            : droneWidthmm(droneWidthmm_), droneHeightmm(droneHeightmm_),
              options_mode{std::move(options_mode_)}, options_onFail{std::move(options_onFail_)},
              options_orientationDimension(options_orientationDimension_),
              options_filter_updatePeriod(options_filter_updatePeriod_), options_filter_movementFreedom(options_filter_movementFreedom_),
              tag_sw{std::move(tag_sw_)}, tag_nw{std::move(tag_nw_)},
              tag_ne{std::move(tag_ne_)}, tag_se{std::move(tag_se_)}
        {}

        Drone(const Drone &) = default;
        Drone(Drone &&) = default;

        Drone & operator =(const Drone & other)
        {
            droneWidthmm = other.droneWidthmm;
            droneHeightmm = other.droneHeightmm;
            options_mode = other.options_mode;
            options_onFail = other.options_onFail;
            options_orientationDimension = other.options_orientationDimension;
            options_filter_updatePeriod = other.options_filter_updatePeriod;
            options_filter_movementFreedom = other.options_filter_movementFreedom;
            tag_sw = other.tag_sw;
            tag_nw = other.tag_nw;
            tag_ne = other.tag_ne;
            tag_se = other.tag_se;

            return *this;
        }
    };

    // Main Settings structure
    class Settings
    {
    public:
        Positioning positioning;
        Drone drone;
        Uwb uwb;

        std::vector<Tag> tags;
        std::vector<Anchor> anchors;

        Settings(Positioning positioning_ = Positioning(), Drone drone_ = Drone(), Uwb uwb_ = Uwb(),
                 std::vector<Tag> tags_ = {}, std::vector<Anchor> anchors_ = {})
            : positioning(positioning_), drone(drone_), uwb(uwb_),
              tags{std::move(tags_)}, anchors{std::move(anchors_)}
        {}

        Settings(const Settings &) = default;
        Settings(Settings &&) = default;

        Settings & operator =(const Settings & other)
        {
            positioning = other.positioning;
            drone = other.drone;
            uwb = other.uwb;

            tags.resize(other.tags.size());
            for(size_t i=0; i < other.tags.size(); i++)
            {
                tags[i] = other.tags[i];
            }

            anchors.resize(other.anchors.size());
            for(size_t i=0; i < other.anchors.size(); i++)
            {
                anchors[i] = other.anchors[i];
            }

            return *this;
        }
    };

    // Basic autocalibration structures
    class AutocalibSettings
    {
    public:
        bool matchPrevious;
        bool testLocalizability;
        std::string method;
        std::string dimension;
        std::string estimator_name;

        AutocalibSettings(bool matchPrevious_ = false, bool testLocalizability_ = true,
                          std::string method_ = "intrarange", std::string dimension_ = "2D", std::string estimator_name_ = "nlls")
            : matchPrevious(matchPrevious_), testLocalizability(testLocalizability_),
              method{std::move(method_)}, dimension{std::move(dimension_)}, estimator_name{std::move(estimator_name_)}
        {}

        AutocalibSettings(const AutocalibSettings &) = default;
        AutocalibSettings(AutocalibSettings &&) = default;
    };

    class AutocalibAnchorConfig
    {
    public:
        std::string id;
        uint8_t order;
        Int3 coordinates;

        AutocalibAnchorConfig(std::string id_ = "22222", uint8_t order_ = 0, Int3 coordinates_ = Int3())
            : id{std::move(id_)}, order(order_), coordinates(coordinates_)
        {}

        AutocalibAnchorConfig(const AutocalibAnchorConfig &) = default;
        AutocalibAnchorConfig(AutocalibAnchorConfig &&) = default;

        AutocalibAnchorConfig & operator =(const AutocalibAnchorConfig & other)
        {
            id = other.id;
            order = other.order;
            coordinates = other.coordinates;

            return *this;
        }
    };

    // Main Autocalibration payload structure
    class Autocalib
    {
    public:
        AutocalibSettings settings;
        std::vector<AutocalibAnchorConfig> anchors;
        std::vector<AutocalibAnchorConfig> oldAnchors;

        Autocalib(AutocalibSettings settings_ = AutocalibSettings(), std::vector<AutocalibAnchorConfig> anchors_ = {},
                  std::vector<AutocalibAnchorConfig> oldAnchors_ = {})
            : settings(settings_), anchors{std::move(anchors_)},
              oldAnchors{std::move(oldAnchors_)}
        {}

        Autocalib(const Autocalib &) = default;
        Autocalib(Autocalib &&) = default;
    };
}
