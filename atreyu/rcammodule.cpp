#include "rcammodule.h"

#include "globalSettings.h"

#include <log.h>
#include <sstream>
#include <limits>
#include "utils.h"

using airt::Log;
using airt::RCamModule;
using airt::StdMessage;
using airt::StdNotification;

RCamModule::RCamModule(const std::string &cmdportname, const std::string &pubportname,
                       std::shared_ptr<Context> context)
    : Module(cmdportname, pubportname, "", context)
{
    assert(context);

    if (!GlobalSettings::getValue("rcam_uart_port", serialport))
    {
        serialport = "/dev/ttyUSB2";
        Log::warn("Using default value rcam_uart_port = {}", serialport);
    }

    if (!GlobalSettings::getValue("rcam_baud_rate", baudrate))
    {
        baudrate = 115200;
        Log::warn("Using default value rcam_baud_rate = {}", baudrate);
    }
}

bool RCamModule::startDevice()
{
    Log::info("RCamModule starts...");
    if (!port.isOpen() && !port.open(serialport, baudrate))
    {
        Log::critical("Failed to open serial port {} at {} bauds", serialport, baudrate);
        throw new std::runtime_error("Error opening serial port");
    }
    airt::sendNotification(*pubsocket, StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::STARTED);
    return true;
};

void RCamModule::stopDevice()
{
    if (port.isOpen())
        port.close();
    Log::info("RCamModule stops...");
    airt::sendNotification(*pubsocket, StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::STOPPED);
};

bool RCamModule::dataReady()
{
    unsigned char buffer[64];
    size_t bytesread;
    while ((bytesread = port.read(buffer, sizeof(buffer))) > 0)
    {
        messageProcessor.ingest(buffer, bytesread);
        if (bytesread < sizeof(buffer))
            break;
    }
    return messageProcessor.hasMessages();
}

void RCamModule::onMessage(const Message &inmsg)
{
    uint8_t command = airt::getMessageAction(inmsg);
    switch (command)
    {
        // Non-payload commands
    case StdMessage::RCAM_SWITCH_TO_REC:
    case StdMessage::RCAM_SWITCH_TO_STILL:
    case StdMessage::RCAM_SWITCH_TO_PB: // change to playback mode
    case StdMessage::RCAM_START_REC:    // start recording
    case StdMessage::RCAM_STOP_REC:     // stop recording
    case StdMessage::RCAM_CAPTURE:      // capture a photo?
    case StdMessage::RCAM_CAPTURE_AF:   // capture a photo with autofocus?
    case StdMessage::RCAM_START_PB:     // start playback
    case StdMessage::RCAM_STOP_PB:      // stop playback
    case StdMessage::RCAM_PAUSE_PB:     // pause playback
    case StdMessage::RCAM_RESUME_PB:    // resume playback
    case StdMessage::RCAM_GET_WIFI:
    case StdMessage::RCAM_GET_BATTERY:                     // request battery charge
    case StdMessage::RCAM_GET_CARD_STATUS:                 // request card status
    case StdMessage::RCAM_GET_MODE:                        // request mode
    case StdMessage::RCAM_GET_STATUS:                      // request status
    case StdMessage::RCAM_GET_REC_REMAINING:               // request remaining recording time
    case StdMessage::RCAM_GET_STILL_REMAINING:             // request number of pictures left
    case StdMessage::RCAM_FORMAT_CARD:                     // format card
    case StdMessage::RCAM_SWITCH_TO_MULTIPLE_MODE_CAPTURE: //
    case StdMessage::RCAM_QUERY_IS_RECORDING:              //
    case StdMessage::RCAM_BURST_CAPTURE_START:             // start burst capture
    case StdMessage::RCAM_BURST_CAPTURE_CANCEL:            // cancel burst capture
    case StdMessage::RCAM_CLEAR_SETTING:                   // clear settings
    case StdMessage::RCAM_INIT_PBMEDIA_FILE:               // init playback media file
    case StdMessage::RCAM_PBMEDIA_NEXT_FILE:               // next playback file
    case StdMessage::RCAM_PBMEDIA_PREV_FILE:               // previous playback file
    case StdMessage::RCAM_PBMEDIA_DELETE_FILE:             // delete file
    case StdMessage::RCAM_SHUTDOWN:
        messageProcessor.sendCommand(port, static_cast<StdMessage::RCamCommandType>(command));
        break;
    case StdMessage::RCAM_SET_WIFI:
        messageProcessor.sendWifiOnOffCommand(port, inmsg.get<const uint8_t *>(0)[3]);
        break;
    case StdMessage::RCAM_EMULATE_KEY:
        messageProcessor.sendEmulateKeyCommand(port, inmsg.get<const uint8_t *>(0)[3]);
        break;
    case StdMessage::RCAM_GET_CONFIG:
        messageProcessor.sendGetConfigCommand(port, static_cast<StdMessage::RCamConfigParameter>(inmsg.get<const uint8_t *>(0)[3]));
        break;
    case StdMessage::RCAM_GET_X_CONFIG:
        messageProcessor.sendGetXConfigCommand(port, static_cast<StdMessage::RCamConfigParameter>(inmsg.get<const uint8_t *>(0)[3]));
        break;
    case StdMessage::RCAM_SET_CONFIG:
        if (!processSetConfigCommand(inmsg))
        {
            airt::sendNotification(*pubsocket, StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::UNKNOWN_COMMAND);
            return;
        }
        break;
    case StdMessage::RCAM_SET_X_CONFIG:
        assert(false);
        airt::sendNotification(*pubsocket, StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::UNKNOWN_COMMAND);
        return;
    default:
        Log::critical("Unhandled message in RCAM module");
        airt::sendNotification(*pubsocket, StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::UNKNOWN_COMMAND);
        return;
    }
    airt::sendNotification(*pubsocket, StdMessage::RCAM_NOTIFICATIONS_MODULE, StdMessage::ACK);
}

bool RCamModule::sendMessage()
{
    while (messageProcessor.hasMessages())
    {
        unsigned char buffer[128];
        unsigned int size;
        if (messageProcessor.popMessage(buffer, sizeof(buffer), size))
        {
            Message msg;
            msg.add_raw(buffer, size);
            pubsocket->send(msg);
        }
        else
        {
            Log::warn("Supposedly there was a message waiting from the serial port");
        }
    }
    return true;
}

bool RCamModule::processSetConfigCommand(const Message &inmsg)
{
    assert(airt::getMessageSignature(inmsg) == 'A');
    assert(airt::getMessageModule(inmsg) == StdMessage::RCAM_MODULE);
    assert(airt::getMessageAction(inmsg) == StdMessage::RCAM_SET_CONFIG);

    auto configParam = inmsg.get<const uint8_t *>(0)[3];
    switch (configParam)
    {
    case StdMessage::RCAM_CONFIG_MOVIE_FORMAT:
    case StdMessage::RCAM_CONFIG_ISO:
    case StdMessage::RCAM_CONFIG_IRIS:
    case StdMessage::RCAM_CONFIG_FOCUS_METHOD:
    case StdMessage::RCAM_CONFIG_LENS_ZOOM:
    case StdMessage::RCAM_CONFIG_PHOTO_SIZE:
    case StdMessage::RCAM_CONFIG_PHOTO_QUALITY:
    case StdMessage::RCAM_CONFIG_WB:
    case StdMessage::RCAM_CONFIG_SHARPNESS:
    case StdMessage::RCAM_CONFIG_FLICKER_REDUCTION:
    case StdMessage::RCAM_CONFIG_AE_METER_MODE:
    case StdMessage::RCAM_CONFIG_WIFI_ONOFF:
    case StdMessage::RCAM_CONFIG_NOISE_REDUCTION:
    case StdMessage::RCAM_CONFIG_LCD_ONOFF:
    case StdMessage::RCAM_CONFIG_ROTATION:
    case StdMessage::RCAM_CONFIG_AF_MODE:
    case StdMessage::RCAM_CONFIG_PHOTO_BURST:
    case StdMessage::RCAM_CONFIG_LED_ENABLE:
    case StdMessage::RCAM_CONFIG_BEEPER_ENABLE:
    case StdMessage::RCAM_CONFIG_PHOTO_BURST_SPEED:

    {
        auto value = inmsg.get<const uint8_t *>(0)[4];
        return messageProcessor.sendSetConfigChoice(port, static_cast<StdMessage::RCamConfigParameter>(configParam), value);
    }
    case StdMessage::RCAM_CONFIG_MF_DRIVE:
    case StdMessage::RCAM_CONFIG_SATURATION:
    case StdMessage::RCAM_CONFIG_BRIGHTNESS:
    case StdMessage::RCAM_CONFIG_CONTRAST:
    case StdMessage::RCAM_CONFIG_EV_BIAS:
    case StdMessage::RCAM_CONFIG_BATTERY:
    case StdMessage::RCAM_CONFIG_AF_AREA:
    case StdMessage::RCAM_CONFIG_LCD_BACKLIGHT_LEVEL:
    case StdMessage::RCAM_CONFIG_MANUAL_WB_TINT:
    {
        const RCamSetConfigRangeValue *rv = inmsg.get<const RCamSetConfigRangeValue *>(0);
        uint8_t buffer[4];
        airt::writeBytesHighToLow(buffer, rv->value);
        return messageProcessor.sendSetConfigRange(port, static_cast<StdMessage::RCamConfigParameter>(configParam),
                                                  buffer , 4);
    }
    case StdMessage::RCAM_CONFIG_SCENE:
    case StdMessage::RCAM_CONFIG_DIGITAL_EFFECT:
    case StdMessage::RCAM_CONFIG_VIDEO_SYSTEM:
    case StdMessage::RCAM_CONFIG_VERSION:

    case StdMessage::RCAM_CONFIG_MAGNIFY_POSITION:
    case StdMessage::RCAM_CONFIG_NEW_FW:
    case StdMessage::RCAM_CONFIG_HW_VERSION:
    case StdMessage::RCAM_CONFIG_DO_AF:
    case StdMessage::RCAM_CONFIG_CAF_ONOFF:
    case StdMessage::RCAM_CONFIG_LENS_ATTACHED:


    case StdMessage::RCAM_CONFIG_MODEL_NAME:

    case StdMessage::RCAM_CONFIG_RTC_TIME:
    case StdMessage::RCAM_CONFIG_BT_MAC:
    case StdMessage::RCAM_CONFIG_MAX_SHUTTER_TIME:
    case StdMessage::RCAM_CONFIG_PC_CONNECTED:
    case StdMessage::RCAM_CONFIG_USB_CABLE_STATUS:
    case StdMessage::RCAM_CONFIG_OLED_ONOFF_ENABLE:
    case StdMessage::RCAM_CONFIG_SHUTTER_ANGLE:
    case StdMessage::RCAM_CONFIG_DCF_REACH_MAX_NUMBER:
    case StdMessage::RCAM_CONFIG_MANUAL_WB:
    case StdMessage::RCAM_CONFIG_HDMI_OSD_ONOFF:
    case StdMessage::RCAM_CONFIG_STILL_SHUTTER_SPEED:
    case StdMessage::RCAM_CONFIG_DCF_FILE_NUMBERING:
    case StdMessage::RCAM_CONFIG_CVBS_VIDEO_SYSTEM:
    case StdMessage::RCAM_CONFIG_CVBS_OUTPUT_ENBLE:
    case StdMessage::RCAM_CONFIG_LENS_FOCUS_POSITION:
    case StdMessage::RCAM_CONFIG_LENS_FOCUS_SPEED:
    case StdMessage::RCAM_CONFIG_CAF_RANGE:
    case StdMessage::RCAM_CONFIG_CAF_SENSITIVITY:
    case StdMessage::RCAM_CONFIG_ENC_ROTATION:
    case StdMessage::RCAM_CONFIG_VIDEO_QUALITY:
    case StdMessage::RCAM_CONFIG_DUAL_STREAM_ENABLE:
    case StdMessage::RCAM_CONFIG_PHOTO_AEB:
    case StdMessage::RCAM_CONFIG_CAPTURE_TIMESTAMP:
    case StdMessage::RCAM_CONFIG_RECORD_TIMESTAMP:
    case StdMessage::RCAM_CONFIG_IMU_ROTATION:
    case StdMessage::RCAM_CONFIG_LUT_TYPE:
    case StdMessage::RCAM_CONFIG_DCF_LAST_FILE_NAME:
    case StdMessage::RCAM_CONFIG_UART_COMMAND_SUPPORTED:
    case StdMessage::RCAM_CONFIG_LCD_RUNTIME_ONOFF:
    case StdMessage::RCAM_CONFIG_MOV_CONTAINER_ROTATION:
    case StdMessage::RCAM_CONFIG_UI_TIMELAPSE_STATUS:
    case StdMessage::RCAM_CONFIG_USB_CHARGE_DETECTION:
    case StdMessage::RCAM_CONFIG_USB_DEVICE_ROLE:
    case StdMessage::RCAM_CONFIG_IC_TEMPERATURE:
    case StdMessage::RCAM_CONFIG_DCF_NAME_MODE:
    case StdMessage::RCAM_CONFIG_RCAM_IS_MULTIPLE:
    case StdMessage::RCAM_CONFIG_DEWARP_ONOFF:
    case StdMessage::RCAM_CONFIG_MAX_RECORD_TEMPERATURE_LIMIT:
    case StdMessage::RCAM_CONFIG_VIGNETTE_ONOFF:
    case StdMessage::RCAM_CONFIG_SECONDRY_STREAM_RESOLUTION:
    case StdMessage::RCAM_CONFIG_SECONDRY_STREAM_BITRATE:
    case StdMessage::RCAM_CONFIG_RECORD_INT_CAP:
    case StdMessage::RCAM_CONFIG_HDMI_PREFER_FORMAT:
    case StdMessage::RCAM_CONFIG_MULTIPLE_CONTROL_ID:
    case StdMessage::RCAM_CONFIG_MULTIPLE_CAPTURE_DELAY:
    case StdMessage::RCAM_CONFIG_RCAM_DEFLECTION_ANGLE:
    case StdMessage::RCAM_CONFIG_VOLUME_CONTROL:
    case StdMessage::RCAM_CONFIG_AE_EXPOSURE_MODE:
    case StdMessage::RCAM_CONFIG_OIS_MODE:
    case StdMessage::RCAM_CONFIG_MOVIE_RECORD_SPLIT_DURATION:
    case StdMessage::RCAM_CONFIG_MULTIPLE_TIMEOUT_ENABLE:
    case StdMessage::RCAM_CONFIG_MULTIPLE_CONTROL_ENABLE:
    case StdMessage::RCAM_CONFIG_AEB_NUMCAPTURE:
    case StdMessage::RCAM_CONFIG_LIVEVIEW_WITH_AUDIO:
    case StdMessage::RCAM_CONFIG_SECONDRY_STREAM_GOP:
    case StdMessage::RCAM_CONFIG_SEND_TO_LNX_STREAM:
    case StdMessage::RCAM_CONFIG_PRIMARY_STREAM_BITRATE:
    case StdMessage::RCAM_CONFIG_SECONDARY_AUDIO_TYPE:
    case StdMessage::RCAM_CONFIG_SECONDARY_B_FRAME:
    case StdMessage::RCAM_CONFIG_AELOCK:
    case StdMessage::RCAM_CONFIG_SECONDARY_STREAM_BITRARE_TYPE: // value is CBR: VBR
    case StdMessage::RCAM_CONFIG_GROUP_INDEX:
    case StdMessage::RCAM_CONFIG_MAX_ISO_LIMIT:
    case StdMessage::RCAM_CONFIG_ETHERNET_IP:
        return false;
    }
    return false;
}