
// This is the public interface to this module. There are actions (requests for the camera) and notifications
// (responses from the camera).
#pragma pack(push, 1)

// ACTIONS
// Non payload actions are built using a single AiRT command packet:
// { 'A', RCAM_MODULE, <ACTION>}
// where <ACTION> can be any of the \sa StdMessage::RCamCommandType that do not require a parameter

// For other actions:

// Powering on/off the wifi:
// { 'A', RCAM_MODULE, RCAM_SET_WIFI, byte{0|1}}

// Emulating a key press. The relationship between id and key is not documented. You will have to figure it out by yourself
// {'A', RCAM_MODULE, RCAM_EMULATE_KEY, byte}

// Getting a configuration value
// {'A', RCAM_MODULE, RCAM_GET_CONFIG, StdMessage::RCamConfigParameter}
struct RCamRequestConfigValue
{
    const AIRT_Message_Header header{StdMessage::RCAM_MODULE, StdMessage::RCAM_GET_CONFIG};
    StdMessage::RCamConfigParameter param;
};

// Getting a X configuration value (what is the difference with the previous? who knows)
// {'A', RCAM_MODULE, RCAM_GET_X_CONFIG, StdMessage::RCamConfigParameter}

// Setting a configuration value
// {'A', RCAM_MODULE, RCAM_SET_CONFIG, StdMessage::RCamConfigParameter, <value>}
// where <value> depends on the config parameter. For example:
//   * set ISO to 1600 (\sa StdMessage::RCamConfigISO)
//    {'A', RCAM_MODULE, RCAM_SET_CONFIG, RCAM_CONFIG_ISO, RCAM_ISO_1600 }
//
//   * set IRIS to F0.7 (\sa StdMessage::RCamConfigIris)
//    {'A', RCAM_MODULE, RCAM_SET_CONFIG, RCAM_CONFIG_IRIS, RCAM_IRIS_F0_7}
//
//   * set FOCUS METHOD to MF (\sa StdMessage::RCamConfigFocusMethod)
//    {'A', RCAM_MODULE, RCAM_SET_CONFIG, RCAM_CONFIG_FOCUS_METHOD, RCAM_FOCUS_MF}
//
//   * set LENS ZOOM to ...
//    {'A', RCAM_MODULE, RCAM_SET_CONFIG, RCAM_CONFIG_LENS_ZOOM, 0}
//
//   * set MF DRIVE to a unsigned int value
//    {'A', RCAM_MODULE, RCAM_SET_CONFIG, RCAM_CONFIG_MF_DRIVE, uint32_t { bytes from most significant to least significant }}

struct RCamSetConfigChoiceValue
{
    const AIRT_Message_Header header{StdMessage::RCAM_MODULE, StdMessage::RCAM_SET_CONFIG};
    StdMessage::RCamConfigParameter param;
    uint8_t value;
};


struct RCamSetConfigRangeValue
{
    const AIRT_Message_Header header{StdMessage::RCAM_MODULE, StdMessage::RCAM_SET_CONFIG};
    StdMessage::RCamConfigParameter param;
    int32_t value;
};


struct RCamEmulateKey
{
    const AIRT_Message_Header header{StdMessage::RCAM_MODULE, StdMessage::RCAM_EMULATE_KEY};
    uint8_t key;
};

// NOTIFICATIONS

// The ZCam informs that the given command failed
struct RCamNackNotification
{
    const AIRT_Message_Header header{StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::RCAM_COMMAND_FAILED_NOTIFICATION};
    uint8_t command;
};
// The Z Cam provides the current battery level
struct RCamBatteryLevelNotification
{
    const AIRT_Message_Header header{StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::RCAM_BATTERY_LEVEL_NOTIFICATION};
    float level; // in percent
};
// Camera mode
struct CameraModeNotification
{
    const AIRT_Message_Header header{StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::RCAM_MODE_NOTIFICATION};
    StdMessage::RCamMode mode;
};

// Camera status
struct CameraStatusNotification
{
    const AIRT_Message_Header header{StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::RCAM_STATUS_NOTIFICATION};
    StdMessage::RCamStatus status;
};

// WIFI status
struct WIFIStatusNotification
{
    const AIRT_Message_Header header{StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::RCAM_WIFI_STATUS_NOTIFICATION};
    uint8_t on; // is the wifi on?
};

// Card status
struct SDCardStatusNotification
{
    const AIRT_Message_Header header{StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::RCAM_CARD_STATUS_NOTIFICATION};
    uint8_t ready; // is the sd card ready?
};

// Recording status
struct RecordingStatusNotification
{
    const AIRT_Message_Header header{StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::RCAM_RECORDING_STATUS_NOTIFICATION};
    uint8_t recording;
};

// Minutes left in recording mode
struct RecordingTimeLeftNotification
{
    const AIRT_Message_Header header{StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::RCAM_REMAINING_REC_TIME_NOTIFICATION};
    uint32_t minutes; // minutes left
};

// Number of photos left in photo mode
struct NumberPhotosLeftNotification
{
    const AIRT_Message_Header header{StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::RCAM_REMAINING_STILL_PHOTOS_NOTIFICATION};
    uint32_t photos; // photos left
};

// Config value status. This is a choice parameter
struct ChoiceConfigParameter
{
    const AIRT_Message_Header header{StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::RCAM_CONFIG_STATUS_NOTIFICATION};
    uint8_t type{0x1}; // This is used to distinguish this parameter from a range parameter
    uint8_t parameter;
    uint8_t value; // Current value
    uint8_t size;  // Number of allowed values for this parameter that follow
    // uint8_t allowed_values[]; // array of the allowed values (the actual size of the message varies depending on the parameter)
};

// Config value status. This is a range parameter
struct RangeConfigParameter
{
    const AIRT_Message_Header header{StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::RCAM_CONFIG_STATUS_NOTIFICATION};
    uint8_t type{0x2}; // This is used to distinguish this parameter from a choice parameter
    uint8_t parameter;
    int32_t value, min, max, step; // Current value, minimum value, maximum value, step
};
#pragma pack(pop)
