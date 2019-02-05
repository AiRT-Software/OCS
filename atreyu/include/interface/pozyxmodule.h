
#pragma pack(push, 1)


// ACTIONS

// N+1 part message for N anchors
struct PositioningAnchorManualConfigHdr
{
    const AIRT_Message_Header header{StdMessage::POSITIONING_MODULE, StdMessage::IPS_ANCHOR_MANUAL_CONFIG};
    uint8_t space;
    int32_t numAnchors;
};
struct PositioningAnchorManualConfig
{
    int32_t id;
    int32_t x, y, z; // mm
    uint8_t order;
};

// N+1 part message for N anchors
struct PositioningAnchorTobeAutocalibratedHdr
{
    const AIRT_Message_Header header{StdMessage::POSITIONING_MODULE, StdMessage::IPS_ANCHOR_TOBE_AUTOCALIBRATED};
    uint8_t space;
    int32_t numAnchors;
};
struct PositioningAnchorTobeAutocalibrated
{
    int id;
    int x, y, z;  // mm
    uint8_t order;  // with the order we know what coordinates have tobe autocalibrated
};

// one part message
struct PositioningDroneTagsManual
{
    const AIRT_Message_Header header{StdMessage::POSITIONING_MODULE, StdMessage::IPS_DRONETAGS_MANUAL_CONFIG};
    int droneWidthmm, droneHeightmm;
    int swID, nwID, neID, seID;
};

// one part message
struct PositioningClearPozyxSettings
{
    const AIRT_Message_Header header{StdMessage::POSITIONING_MODULE, StdMessage::IPS_CLEAR_POZYXSETTINGS};
};

// one part message
struct PositioningSetDroneFilter
{
    const AIRT_Message_Header header{StdMessage::POSITIONING_MODULE, StdMessage::IPS_SET_DRONEFILTER};
    uint8_t space;
    float updatePeriod;  // seconds ]0, 1], default 0.2
    int32_t movementFreedom;  // ]0, 1000], default 50
};

// NOTIFICATIONS

//  Watch out! Don't use this message. The roll and pitch is always zero. Use the position/orientation provided by the DroneModule
// (see interface/dronemodule.h)
struct PositioningFrame
{
    const AIRT_Message_Header header{StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_DATA};
    uint8_t space; // not used 
    double timestamp;
    float pitch, roll, yaw;
    float x, y, z;  // mm 
};

// N+1 part message for N discovered anchors or (loaded from settings anchors)
struct positioningAnchorLocationHeader
{
    const AIRT_Message_Header header{StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_DATA_ANCHOR};
    uint8_t space;
    int32_t numAnchors;
};
struct positioningAnchorLocation
{
    int id;
    int x, y, z; //mm
    uint8_t order;
};

// N+1 part message for N discovered anchors
struct positioningAnchorsListHdr
{
    const AIRT_Message_Header header{StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_DATA_ANCHORS_LIST};
    int numAnchors;
};
struct positioningAnchorsListItem
{
    int id;
};

// 1st part message
struct PositioningDroneTagsListHdr
{
    const AIRT_Message_Header header{StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_DRONETAGS_LIST};
    uint8_t numTags;  // it is variable
};
// 2nd part message (array of ints)
struct PositioningDroneTagsListItem
{
    int id;
};

// one part message
struct PositioningDroneTags
{
    const AIRT_Message_Header header{StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_DRONETAGS};
    int droneWidthmm, droneHeightmm;
    int swID, nwID, neID, seID;
};

// one part message
struct PositioningGetDroneFilterNotification
{
    const AIRT_Message_Header header{StdMessage::POSITIONING_NOTIFICATIONS_MODULE, StdMessage::IPS_GET_DRONEFILTER_NOTIFICATION};
    uint8_t space;
    float updatePeriod;  // seconds ]0, 1], default 0.2
    int32_t movementFreedom;  // ]0, 1000], default 50
};

inline void toCSV(std::ostream &os, const PositioningFrame &ips_frame)
{
    os << std::setprecision(16) << ips_frame.timestamp << std::setprecision(6) << ";"
       << ips_frame.x << ";" << ips_frame.y << ";" << ips_frame.z << ";" << ips_frame.yaw
       << ";";
}

inline void toCSV(std::ostream &os, const positioningAnchorLocation &ips_anchor_location)
{
    os << ips_anchor_location.id << ";"
       << ips_anchor_location.order << ";"
       << ips_anchor_location.x << ";" << ips_anchor_location.y << ";" << ips_anchor_location.z
       << ";";
}
#pragma pack(pop)
