#include "zCamE1MessageProcessor.h"
#include "../atreyu/include/rcammodule.h"
#include "stdMessage.h"
#ifdef TESTING
#include "dummySerial.h"
#else
#include "serial.h"
#endif

#include "log.h"
#include "utils.h"

#define MAX_MSG_LENGTH 32

// Common indices
#define SYNC1_INDEX 0
#define SYNC2_INDEX 1
#define LENGTH_INDEX 2
#define CMD_INDEX 3

// Indices of elements in a command
#define CMD_PAYLOAD_INDEX 4

// Indices of elements in a notification
#define NTF_OK_INDEX 4
#define NTF_PAYLOAD_INDEX 5

#define CONFIG_PARAM_TYPE_INDEX 5

// Sync bytes
#define SYNC1 0xea
#define SYNC2 0x02

/*

Z Cam E1 packet format.
See https://github.com/imaginevision/Z-Camera-Doc/blob/master/expansion/expansion.md

Commands have the following format:

	req: 0xea | 0x02 | len | cmd | param0 | param1 | param2 | ...

  len = sizeof(cmd) + sizeof(params)

Notifications have the following format:

	ack: 0xea | 0x02 | len | (cmd|0x80) | {ok/ng} | param, if it has

	len = sizeof(cmd) + sizeof(param) + sizeof(ok/ng)

  ok: 0
  ng: !0 (no go?)

	e.g.
		switch mode to rec command: 0x02
		req: 0xea 0x02 0x01 0x02
		ack: 0xea 0x02 0x02 0x82 0x00

*/
enum Commands
{
  UART_CMD_NONE,
  UART_EMULATE_KEY,
  UART_SWITCH_TO_REC,
  UART_SWITCH_TO_STILL,
  UART_SWITCH_TO_PB,
  UART_START_REC, // 5
  UART_STOP_REC,
  UART_CAPTURE,
  UART_CAPTURE_AF,
  UART_AF,       // deprecated
  UART_START_PB, // 0xa
  UART_STOP_PB,
  UART_PAUSE_PB,
  UART_RESUME_PB,
  UART_SET_CONFIG,
  UART_GET_CONFIG,
  UART_SET_WIFI, // 0x10
  UART_GET_WIFI,
  UART_GET_BATTERY,
  UART_GET_CARD_STATUS,
  UART_GET_MODE,
  UART_GET_STATUS, // 0x15
  UART_GET_REC_REMAINING,
  UART_GET_STILL_REMAINING,
  UART_FORMAT_CARD,
  UART_GET_BT_VERSION,                  // deprecated
  UART_SWITCH_TO_MULTIPLE_MODE_CAPTURE, // 0x1a
  UART_SET_X_CONFIG,
  UART_GET_X_CONFIG,
  UART_QUERY_IS_RECORDING,
  UART_BURST_CAPTURE_START,
  UART_BURST_CAPTURE_CANCEL, // 0x1f
  UART_CLEAR_SETTING,
  UART_INIT_PBMEDIA_FILE,
  UART_PBMEDIA_NEXT_FILE,
  UART_PBMEDIA_PREV_FILE,
  UART_PBMEDIA_DELETE_FILE,
  UART_SHUTDOWN // 0x25
};

using airt::StdMessage;

// From Z Cam codes -> AiRT
static unsigned char ZCAMtoAirtCommands[]{
    StdMessage::RCAM_COMMAND_FAILED_NOTIFICATION,
    StdMessage::RCAM_EMULATE_KEY_NOTIFICATION,       // emulate a key press
    StdMessage::RCAM_SWITCHED_TO_REC_NOTIFICATION,   // changed to record mode (video)
    StdMessage::RCAM_SWITCHED_TO_STILL_NOTIFICATION, // changed to photo mode
    StdMessage::RCAM_SWITCHED_TO_PB_NOTIFICATION,    // change to playback mode
    StdMessage::RCAM_STARTED_REC_NOTIFICATION,       // started recording
    StdMessage::RCAM_STOPPED_REC_NOTIFICATION,       // stop recording
    StdMessage::RCAM_CAPTURED_NOTIFICATION,          // captured a photo
    StdMessage::RCAM_CAPTURED_AF_NOTIFICATION,       // capture a photo with autofocus?
    StdMessage::RCAM_COMMAND_FAILED_NOTIFICATION,    // Deprecated
    StdMessage::RCAM_STARTED_PB_NOTIFICATION,        // start playback

    StdMessage::RCAM_STOPPED_PB_NOTIFICATION,     // stop playback
    StdMessage::RCAM_PAUSED_PB_NOTIFICATION,      // paused
    StdMessage::RCAM_RESUMED_PB_NOTIFICATION,     // resumed
    StdMessage::RCAM_CONFIG_CHANGED_NOTIFICATION, // set configuration
    StdMessage::RCAM_CONFIG_STATUS_NOTIFICATION,  // | cmd | key |
    StdMessage::RCAM_WIFI_SET_NOTIFICATION,
    StdMessage::RCAM_WIFI_STATUS_NOTIFICATION,
    StdMessage::RCAM_BATTERY_LEVEL_NOTIFICATION, // request battery charge
    StdMessage::RCAM_CARD_STATUS_NOTIFICATION,   // request card status
    StdMessage::RCAM_MODE_NOTIFICATION,          // request mode

    StdMessage::RCAM_STATUS_NOTIFICATION,                            // camera status (see RCamStatus below)
    StdMessage::RCAM_REMAINING_REC_TIME_NOTIFICATION,                // request remaining recording time
    StdMessage::RCAM_REMAINING_STILL_PHOTOS_NOTIFICATION,            // request number of pictures left
    StdMessage::RCAM_CARD_FORMATTED_NOTIFICATION,                    // format card
    StdMessage::RCAM_COMMAND_FAILED_NOTIFICATION,                    // Deprecated
    StdMessage::RCAM_SWITCHED_TO_MULTIPLE_MODE_CAPTURE_NOTIFICATION, //
    StdMessage::RCAM_CHANGED_X_CONFIG_NOTIFICATION,                  //
    StdMessage::RCAM__X_CONFIG_NOTIFICATION,                         //
    StdMessage::RCAM_RECORDING_STATUS_NOTIFICATION,                  //
    StdMessage::RCAM_BURST_CAPTURE_STARTED_NOTIFICATION,             // start burst capture

    StdMessage::RCAM_BURST_CAPTURE_CANCELED_NOTIFICATION,    // cancel burst capture
    StdMessage::RCAM_SETTING_CLEARED_NOTIFICATION,           // clear settings
    StdMessage::RCAM_PBMEDIA_FILE_INITED_NOTIFICATION,       // init playback media file
    StdMessage::RCAM_PBMEDIA_CHANGED_NEXT_FILE_NOTIFICATION, // next playback file
    StdMessage::RCAM_PBMEDIA_CHANGED_PREV_FILE_NOTIFICATION, // previous playback file
    StdMessage::RCAM_PBMEDIA_DELETED_FILE_NOTIFICATION,      // delete file
    StdMessage::RCAM_SHUTTING_DOWN_NOTIFICATION              // shutdown
};

using airt::ZCamE1MessageProcessor;

ZCamE1MessageProcessor::ZCamE1MessageProcessor() : inbuffer(bufferCapacity), state(Status::FIRST_SYNC), discarded_bytes(0),
                                                   payload_read_bytes(0), config_requests(16)
{
  static_assert(static_cast<uint8_t>(StdMessage::RCAM_SHUTTING_DOWN_NOTIFICATION) - static_cast<uint8_t>(UART_SHUTDOWN) ==
                    static_cast<uint8_t>(StdMessage::RCAM_EMULATE_KEY_NOTIFICATION) - static_cast<uint8_t>(UART_EMULATE_KEY),
                "The elements in the Commands structure above should coincide one-to-one with the RCamNotificationType in MessageCodes.cs");
}

bool ZCamE1MessageProcessor::ingest(unsigned char *data, unsigned int size)
{
  if (size + inbuffer.size() > inbuffer.capacity())
    return false;

  inbuffer.insert(inbuffer.end(), data, data + size);
  return true;
}

bool ZCamE1MessageProcessor::hasMessages()
{
  if (state == Status::DONE)
    return true;

  return parse();
}

void ZCamE1MessageProcessor::restart()
{
  discarded_bytes += message.size();
  message.clear();
  state = Status::FIRST_SYNC;
  payload_read_bytes = 0;
}

bool ZCamE1MessageProcessor::popMessage(unsigned char *buffer, const unsigned int max, unsigned int &size)
{
  if (!hasMessages())
    return false;

  assert(message.size() < max);

  translateNotification(buffer, max, size);
  state = Status::FIRST_SYNC;
  message.clear();
  payload_read_bytes = 0;

  return true;
}

bool ZCamE1MessageProcessor::parse()
{
  while (!inbuffer.empty() && state != Status::DONE)
  {
    unsigned char c = inbuffer[0];
    inbuffer.pop_front();
    message.push_back(c);

    switch (state)
    {
    case Status::FIRST_SYNC:
      if (c == SYNC1) {
        state = Status::SECOND_SYNC;
      } else
      {
        restart();
#ifdef _DEBUG        
        Log::error("Discarded bytes SYNC1: {} ({})", discarded_bytes, c);
#endif

      }
      break;
    case Status::SECOND_SYNC:
      if (c == SYNC2)
        state = Status::LENGTH;
      else {
        restart();
#ifdef _DEBUG
        Log::error("Discarded bytes SYNC2: {} ({})", discarded_bytes, c);
#endif
      }
      break;
    case Status::LENGTH:
      state = Status::CMD;
      break;
    case Status::CMD:
      if (c < 0x80) {
        restart();
#ifdef _DEBUG
        Log::error("Discarded bytes CMD: {} ({})", discarded_bytes, c);
#endif
      }
      else
        state = Status::OK;
      break;
    case Status::OK:
      if (message[LENGTH_INDEX] <= 2)
        state = Status::DONE;
      else
      {
        state = Status::PAYLOAD;
        payload_read_bytes = 0;
      }
      break;
    case Status::PAYLOAD:
      payload_read_bytes++;
      if (payload_read_bytes >= (message[LENGTH_INDEX] - 2U))
      {
        state = Status::DONE;
        Log::info("Received msg from serial: {} ({} bytes)", message[CMD_INDEX], message.size());
      }
      break;
    case Status::DONE:
      assert(false);
      break;
    }
  }
  return state == Status::DONE;
}

template <typename C>
void copyMessage(C *msg, unsigned char *buffer, unsigned int &size)
{
  memcpy(buffer, msg, sizeof(C));
  size = sizeof(C);
}

void ZCamE1MessageProcessor::translateNotification(unsigned char *buffer, const unsigned int max, unsigned int &size)
{
  auto command = message[CMD_INDEX] & 0x7F;
  // First, check if the notification is an error
  if (message[NTF_OK_INDEX] != 0 || message[CMD_INDEX] == UART_AF || message[CMD_INDEX] == UART_GET_BT_VERSION)
  {
    airt::RCamModule::RCamNackNotification m;
    m.command = command;
    copyMessage(&m, buffer, size);
    if (m.command == UART_GET_CONFIG)
      config_requests.pop_front();
    return;
  }

  switch (command)
  {
  // Notifications without payload
  case UART_EMULATE_KEY:
  case UART_SWITCH_TO_REC:
  case UART_SWITCH_TO_STILL:
  case UART_SWITCH_TO_PB:
  case UART_START_REC:
  case UART_STOP_REC:
  case UART_CAPTURE:
  case UART_CAPTURE_AF:
  case UART_START_PB:
  case UART_STOP_PB:
  case UART_PAUSE_PB:
  case UART_RESUME_PB:
  case UART_SET_CONFIG:
  case UART_FORMAT_CARD:
  case UART_SET_WIFI:
  case UART_SWITCH_TO_MULTIPLE_MODE_CAPTURE:
  case UART_SET_X_CONFIG:
  case UART_BURST_CAPTURE_START:
  case UART_BURST_CAPTURE_CANCEL:
  case UART_CLEAR_SETTING:
  case UART_INIT_PBMEDIA_FILE:
  case UART_PBMEDIA_NEXT_FILE:
  case UART_PBMEDIA_PREV_FILE:
  case UART_PBMEDIA_DELETE_FILE:
  case UART_SHUTDOWN:
  {
    airt::AIRT_Message_Header m(StdMessage::RCAM_NOTIFICATIONS_MODULE, ZCAMtoAirtCommands[command]);
    copyMessage(&m, buffer, size);
  }
  break;
  case UART_GET_WIFI:
  {
    airt::RCamModule::WIFIStatusNotification m;
    m.on = message[NTF_PAYLOAD_INDEX];
    copyMessage(&m, buffer, size);
  }
  break;
  case UART_GET_MODE:
  {
    airt::RCamModule::CameraModeNotification m;
    m.mode = static_cast<StdMessage::RCamMode>(message[NTF_PAYLOAD_INDEX]);
    copyMessage(&m, buffer, size);
  }
  break;
  case UART_GET_CARD_STATUS:
  {
    airt::RCamModule::SDCardStatusNotification m;
    m.ready = message[NTF_PAYLOAD_INDEX];
    copyMessage(&m, buffer, size);
  }
  break;
  case UART_QUERY_IS_RECORDING:
  {
    airt::RCamModule::RecordingStatusNotification m;
    m.recording = message[NTF_PAYLOAD_INDEX];
    copyMessage(&m, buffer, size);
  }
  break;
  case UART_GET_STATUS:
  {
    airt::RCamModule::CameraStatusNotification m;
    m.status = static_cast<StdMessage::RCamStatus>(message[NTF_PAYLOAD_INDEX]);
    copyMessage(&m, buffer, size);
  }
  break;
  case UART_GET_REC_REMAINING:
  {
    airt::RCamModule::RecordingTimeLeftNotification m;
    assert(message[LENGTH_INDEX] == 6);
    readBytesHighToLow(&message[NTF_PAYLOAD_INDEX], m.minutes);
    copyMessage(&m, buffer, size);
  }
  break;
  case UART_GET_STILL_REMAINING:
  {
    airt::RCamModule::NumberPhotosLeftNotification m;
    assert(message[LENGTH_INDEX] == 6);
    readBytesHighToLow(&message[NTF_PAYLOAD_INDEX], m.photos);
    copyMessage(&m, buffer, size);
  }
  break;
  case UART_GET_CONFIG:
    translateConfigNotification(buffer, max, size);
    break;
  case UART_GET_X_CONFIG:
    Log::critical("We don't know how to handle UART_GET_X_CONFIG commands");
    size = 0;
    assert(false);
    break;
  case UART_GET_BATTERY:
  {
    airt::RCamModule::RCamBatteryLevelNotification m;
    m.level = message[NTF_PAYLOAD_INDEX];
    copyMessage(&m, buffer, size);
  }
  break;
  }
}

void ZCamE1MessageProcessor::translateConfigNotification(unsigned char *buffer, const unsigned int max, unsigned int &size)
{
  auto oldest_request = config_requests.front();
  config_requests.pop_front();

  if (message[CONFIG_PARAM_TYPE_INDEX] == 0x1)
  {
    // if type is 0x01(choice), | 0xea | 0x02 | len | ack_cmd | ok/ng | type | current config | config list |
    //                             0       1     2       3        4       5          6               7
    assert(message.size() >= 7);

    airt::RCamModule::ChoiceConfigParameter m;
    m.type = message[CONFIG_PARAM_TYPE_INDEX];
    m.parameter = oldest_request;
    m.value = message[6];
    m.size = message.size() - 2 /* syncs */ - 1 /* len */ - 1 /* ack_cmd */ - 1 /* ok/ng */ - 1 /* type */ - 1 /*current config */;

    copyMessage(&m, buffer, size);
    for (unsigned int i = 0; i < m.size; i++)
    {
      buffer[size] = message[7 + i];
      size++;
    }
  }
  else if (message[CONFIG_PARAM_TYPE_INDEX] == 0x2)
  {
    // if type is 0x02(range), | 0xea | 0x02 | len | ack_cmd | ok/ng | type | current value (4 bytes, high|low) | max (4 bytes, hig|low) | min (4 bytes, high|low) | step (4 bytes, high|low) |
    assert(message.size() == (6 + 4 * 4));

    airt::RCamModule::RangeConfigParameter m;
    m.type = message[CONFIG_PARAM_TYPE_INDEX];
    m.parameter = oldest_request;
    readBytesHighToLow(&message[6], m.value);
    readBytesHighToLow(&message[10], m.max);
    readBytesHighToLow(&message[14], m.min);
    readBytesHighToLow(&message[18], m.step);
    copyMessage(&m, buffer, size);
  }
  else
  {
    Log::critical("Unsupported parameter type {}", message[CONFIG_PARAM_TYPE_INDEX]);
    airt::RCamModule::RCamNackNotification m;
    m.command = StdMessage::RCAM_GET_CONFIG;
    copyMessage(&m, buffer, size);
  }
}

bool ZCamE1MessageProcessor::sendZCAMCommand(Serial &port, unsigned char c)
{
  unsigned char msg[]{SYNC1, SYNC2, 0x1, c};
  return port.write(msg, sizeof(msg)) == sizeof(msg);
}

bool ZCamE1MessageProcessor::sendZCAMCommand(Serial &port, unsigned char c, unsigned char *payload, unsigned int size)
{
  unsigned int totalMsgSize = size + 2 + 1 + 1; // 2 syncs + 1 length + 1 cmd

  assert(totalMsgSize < MAX_MSG_LENGTH);

  unsigned char buffer[MAX_MSG_LENGTH];
  buffer[SYNC1_INDEX] = SYNC1;
  buffer[SYNC2_INDEX] = SYNC2;
  buffer[LENGTH_INDEX] = size + 1;
  buffer[CMD_INDEX] = c;
  memcpy(buffer + CMD_PAYLOAD_INDEX, payload, size);
  Log::info("Sending to ZCam command {} ({} bytes)", c, totalMsgSize);
  return port.write(buffer, totalMsgSize) == totalMsgSize;
}

// Send the command through the serial port: no payload
bool ZCamE1MessageProcessor::sendCommand(Serial &port, StdMessage::RCamCommandType c)
{
  switch (static_cast<uint8_t>(c))
  {
  case StdMessage::RCAM_SWITCH_TO_REC:
    return sendZCAMCommand(port, UART_SWITCH_TO_REC);
    break;
  case StdMessage::RCAM_SWITCH_TO_STILL:
    return sendZCAMCommand(port, UART_SWITCH_TO_STILL);
    break;
  case StdMessage::RCAM_SWITCH_TO_PB:
    return sendZCAMCommand(port, UART_SWITCH_TO_PB);
    break;
  case StdMessage::RCAM_START_REC:
    return sendZCAMCommand(port, UART_START_REC);
    break;
  case StdMessage::RCAM_STOP_REC:
    return sendZCAMCommand(port, UART_STOP_REC);
    break;
  case StdMessage::RCAM_CAPTURE:
    return sendZCAMCommand(port, UART_CAPTURE);
    break;
  case StdMessage::RCAM_CAPTURE_AF:
    return sendZCAMCommand(port, UART_CAPTURE_AF);
    break;
  case StdMessage::RCAM_START_PB:
    return sendZCAMCommand(port, UART_START_PB);
    break;
  case StdMessage::RCAM_STOP_PB:
    return sendZCAMCommand(port, UART_STOP_PB);
    break;
  case StdMessage::RCAM_PAUSE_PB:
    return sendZCAMCommand(port, UART_PAUSE_PB);
    break;
  case StdMessage::RCAM_RESUME_PB:
    return sendZCAMCommand(port, UART_RESUME_PB);
    break;
  case StdMessage::RCAM_GET_BATTERY:
    return sendZCAMCommand(port, UART_GET_BATTERY);
    break;
  case StdMessage::RCAM_GET_CARD_STATUS:
    return sendZCAMCommand(port, UART_GET_CARD_STATUS);
    break;
  case StdMessage::RCAM_GET_MODE:
    return sendZCAMCommand(port, UART_GET_MODE);
    break;
  case StdMessage::RCAM_GET_STATUS:
    return sendZCAMCommand(port, UART_GET_STATUS);
    break;
  case StdMessage::RCAM_GET_REC_REMAINING:
    return sendZCAMCommand(port, UART_GET_REC_REMAINING);
    break;
  case StdMessage::RCAM_GET_STILL_REMAINING:
    return sendZCAMCommand(port, UART_GET_STILL_REMAINING);
    break;
  case StdMessage::RCAM_FORMAT_CARD:
    return sendZCAMCommand(port, UART_FORMAT_CARD);
    break;
  case StdMessage::RCAM_SWITCH_TO_MULTIPLE_MODE_CAPTURE:
    return sendZCAMCommand(port, UART_SWITCH_TO_MULTIPLE_MODE_CAPTURE);
    break;
  case StdMessage::RCAM_QUERY_IS_RECORDING:
    return sendZCAMCommand(port, UART_QUERY_IS_RECORDING);
    break;
  case StdMessage::RCAM_BURST_CAPTURE_START:
    return sendZCAMCommand(port, UART_BURST_CAPTURE_START);
    break;
  case StdMessage::RCAM_BURST_CAPTURE_CANCEL:
    return sendZCAMCommand(port, UART_BURST_CAPTURE_CANCEL);
    break;
  case StdMessage::RCAM_CLEAR_SETTING:
    return sendZCAMCommand(port, UART_CLEAR_SETTING);
    break;
  case StdMessage::RCAM_INIT_PBMEDIA_FILE:
    return sendZCAMCommand(port, UART_INIT_PBMEDIA_FILE);
    break;
  case StdMessage::RCAM_PBMEDIA_NEXT_FILE:
    return sendZCAMCommand(port, UART_PBMEDIA_NEXT_FILE);
    break;
  case StdMessage::RCAM_PBMEDIA_PREV_FILE:
    return sendZCAMCommand(port, UART_PBMEDIA_PREV_FILE);
    break;
  case StdMessage::RCAM_PBMEDIA_DELETE_FILE:
    return sendZCAMCommand(port, UART_PBMEDIA_DELETE_FILE);
    break;
  case StdMessage::RCAM_SHUTDOWN:
    return sendZCAMCommand(port, UART_SHUTDOWN);
    break;
  case StdMessage::RCAM_SET_CONFIG:
  case StdMessage::RCAM_GET_CONFIG:
  case StdMessage::RCAM_SET_X_CONFIG:
  case StdMessage::RCAM_GET_X_CONFIG:
    assert(false);
    Log::critical("This ZCam command {} requires parameters. Use the other sendXXXXCommand methods.", c);
    return false;
    break;
  default:
    return false;
  }
}

bool ZCamE1MessageProcessor::sendWifiOnOffCommand(Serial &port, bool on)
{
  uint8_t wifion = on ? 1 : 0;
  return sendZCAMCommand(port, UART_SET_WIFI, &wifion, 1);
}

bool ZCamE1MessageProcessor::sendEmulateKeyCommand(Serial &port, uint8_t key)
{
  return sendZCAMCommand(port, UART_EMULATE_KEY, &key, 1);
}

bool ZCamE1MessageProcessor::sendGetConfigCommand(Serial &port, StdMessage::RCamConfigParameter setting)
{
  uint8_t s = static_cast<uint8_t>(setting);
  config_requests.push_back(s);
  return sendZCAMCommand(port, UART_GET_CONFIG, &s, 1);
}

bool ZCamE1MessageProcessor::sendGetXConfigCommand(Serial &port, StdMessage::RCamConfigParameter setting)
{
  uint8_t s = static_cast<uint8_t>(setting);
  return sendZCAMCommand(port, UART_GET_X_CONFIG, &s, 1);
}

bool ZCamE1MessageProcessor::sendSetConfigChoice(Serial &port, StdMessage::RCamConfigParameter setting, uint8_t value)
{
  //if type is 0x01(choice), | 0xea | 0x02 | len | set_config | config key | type | current config |
  uint8_t payload[]{setting, 0x1, value};
  return sendZCAMCommand(port, UART_SET_CONFIG, payload, sizeof(payload));
}

bool ZCamE1MessageProcessor::sendSetConfigRange(Serial &port, StdMessage::RCamConfigParameter setting, const uint8_t *value, size_t size)
{
  //if type is 0x02(range), | 0xea | 0x02 | len | set_config | config key | type | current value （4 bytes, high|low）|
  uint8_t payload[size + 2]{setting, 0x2};
  memcpy(payload + 2, value, size);
  return sendZCAMCommand(port, UART_SET_CONFIG, payload, sizeof(payload));
}
