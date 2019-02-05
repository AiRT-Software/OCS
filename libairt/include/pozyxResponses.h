#pragma once
#include "pozyxPayloads.h"


namespace PozyxResponses
{
    class System
    {
    public:
        std::string status;
        std::string version;
        std::string name;
        PozyxPayloads::Device device;

        System(std::string status_ = "", std::string version_ = "", std::string name_ = "",
               PozyxPayloads::Device device_ = PozyxPayloads::Device())
            : status{std::move(status_)}, version{std::move(version_)}, name{std::move(name_)},
              device(device_)
        {}

        System(const System &) = default;
        System(System &&) = default;
    };

    class Discover
    {
    public:
        bool success;
        std::vector<PozyxPayloads::Tag> tags;
        std::vector<PozyxPayloads::Anchor> anchors;

        Discover(bool success_ = false, std::vector<PozyxPayloads::Tag> tags_ = {}, std::vector<PozyxPayloads::Anchor> anchors_ = {})
            : success(success_), tags{std::move(tags_)}, anchors{std::move(anchors_)}
        {}

        Discover(const Discover &) = default;
        Discover(Discover &) = default;
    };

    class AutocalibratedAnchor
    {
    public:
        std::string id;
        uint8_t order;
        PozyxPayloads::Int3 coordinates;

        AutocalibratedAnchor(std::string id_ = "", uint8_t order_ = 0, PozyxPayloads::Int3 coordinates_ = PozyxPayloads::Int3())
            : id{std::move(id_)}, order(order_), coordinates(coordinates_)
        {}

        AutocalibratedAnchor(const AutocalibratedAnchor &) = default;
        AutocalibratedAnchor(AutocalibratedAnchor &) = default;

        AutocalibratedAnchor & operator =(const AutocalibratedAnchor & other)
        {
            id = other.id;
            order = other.order;
            coordinates = other.coordinates;
            return *this;
        }
    };

    class AutocalibrationWarning
    {
    public:
        std::string type;
        std::string pair_id1;
        std::string pair_id2;

        AutocalibrationWarning(std::string type_ = "", std::string pair_id1_ = "", std::string pair_id2_ = "")
            : type{std::move(type_)}, pair_id1{std::move(pair_id1_)}, pair_id2{std::move(pair_id2_)}
        {}

        AutocalibrationWarning(const AutocalibrationWarning &) = default;
        AutocalibrationWarning(AutocalibrationWarning &) = default;
    };

    class Autocalibrate
    {
    public:
        bool success;
        int score;
        int connectivity;
        std::vector<AutocalibrationWarning> warnings;
        std::vector<AutocalibratedAnchor> anchors;

        Autocalibrate(bool success_ = false, int score_ = 0, int connectivity_ = 0,
                      std::vector<AutocalibrationWarning> warnings_ = {},
                      std::vector<AutocalibratedAnchor> anchors_ = {})
            : success(success_), score(score_), connectivity(connectivity_),
              warnings{std::move(warnings_)},
              anchors{std::move(anchors_)}
        {}

        Autocalibrate(const Autocalibrate &) = default;
        Autocalibrate(Autocalibrate &) = default;
    };

    class UpdatedDevice
    {
    public:
        std::string id;
        bool success;
        std::string error;

        UpdatedDevice(std::string id_ = "", bool success_ = false, std::string error_ = "")
            : id{std::move(id_)}, success(success_), error{std::move(error_)}
        {}

        UpdatedDevice(const UpdatedDevice &) = default;
        UpdatedDevice(UpdatedDevice &) = default;
    };

    class UpdateSettings
    {
    public:
        bool success;
        std::string error_general;
        std::vector<UpdatedDevice> tags_updated;
        std::vector<UpdatedDevice> anchors_updated;

        UpdateSettings(bool success_ = false, std::string error_general_ = "",
                       std::vector<UpdatedDevice> tags_updated_ = {}, std::vector<UpdatedDevice> anchors_updated_ = {})
            : success(success_), error_general{std::move(error_general_)},
              tags_updated{std::move(tags_updated_)}, anchors_updated{std::move(anchors_updated_)}
        {}

        UpdateSettings(const UpdateSettings &) = default;
        UpdateSettings(UpdateSettings &) = default;
    };

    class RangedDevice
    {
    public:
        std::string anchorId;
        int range;

        RangedDevice(std::string anchorId_ = "11111", int range_ = 0)
            : anchorId{std::move(anchorId_)}, range(range_)
        {}

        RangedDevice(const RangedDevice &) = default;
        RangedDevice(RangedDevice &) = default;
    };

    class PositioningFrame
    {
    public:
        std::string version;
        std::string tagId;
        bool success;
        std::string error;
        double timestamp;

        PozyxPayloads::Float3 coordinates;
        PozyxPayloads::Float3 velocity;
        float roll, pitch, yaw;
        PozyxPayloads::Float3 north;

        std::vector<RangedDevice> ranges;

        float latency;
        float rate_update;
        float rate_success;

        float positioning_latency;
        float drone_calculation_latency;

        std::vector<int> tags_ids;

        PositioningFrame(std::string version_ = "", std::string tagId_ = "", bool success_ = false, std::string error_ = "", double timestamp_ = 0.0,
                         PozyxPayloads::Float3 coordinates_ = PozyxPayloads::Float3(), PozyxPayloads::Float3 velocity_ = PozyxPayloads::Float3(),
                         float roll_ = 0.0, float pitch_ = 0.0, float yaw_ = 0.0, PozyxPayloads::Float3 north_ = PozyxPayloads::Float3(),
                         std::vector<RangedDevice> ranges_ = {},
                         float latency_ = 0.0, float rate_update_ = 0.0, float rate_succes_ = 0.0,
                         float positioning_latency_ = 0.0, float drone_calculation_latency_ = 0.0,
                         std::vector<int> tags_ids_ = {}
                )
            : version{std::move(version_)}, tagId{std::move(tagId_)}, success(success_), error{std::move(error_)}, timestamp(timestamp_),
              coordinates(coordinates_), velocity(velocity_),
              roll(roll_), pitch(pitch_), yaw(yaw_), north(north_),
              ranges{std::move(ranges_)},
              latency(latency_), rate_update(rate_update_), rate_success(rate_succes_),
              positioning_latency(positioning_latency_), drone_calculation_latency(drone_calculation_latency_),
              tags_ids{std::move(tags_ids_)}
        {}

        PositioningFrame(const PositioningFrame &) = default;
        PositioningFrame(PositioningFrame &) = default;
    };

    class ShortCommandResp
    {
    public:
        bool success;
        std::string error;

        ShortCommandResp(bool success_ = false, std::string error_ = "")
            : success(success_), error{std::move(error_)}
        {}

        ShortCommandResp(const ShortCommandResp &) = default;
        ShortCommandResp(ShortCommandResp &) = default;
    };
}
