#pragma once

#include <boost/circular_buffer.hpp>
#include "stdMessage.h"
namespace airt
{

class Serial;
// class StdMessage;
// enum StdMessage::RCamCommandType : uint8_t;
// enum StdMessage::RCamConfigParameter : uint8_t;

/**
 * \class ZCamE1MessageProcessor
 * This class is in charge of translating between AiRT and Aerotools multiplexer protocols.
 * There are two kinds of messages:
 * -Commands: sent from AiRT to Aerotools multiplexer
 * -Notifications: sent from Aerotools multiplexer to AiRT
 * 
 * In order to add more commands:
 * 
 * 1. Add a new constant to the Commands enum below
 * 2. If the command has no payload, you are done. The new command can be sent with the 
 *  AeroToolsFCSMessageProcessor::sendCommand
 * 3. If the command has payload, add a new method called sendXXXCommand, where XXX defines 
 * the type of command and it has the required parameters bo build the Aerotools message.
 * 
 * In order to add more notifications:
 * 1. Add a new constant to the Notificatios enum below
 * 2. Add a new case to the switch in the AeroToolsFCSMessageProcessor::translateMessage method to
 * translate the message into the AiRT notification
*/

class ZCamE1MessageProcessor
{
public:
  ZCamE1MessageProcessor();

  /**
     * 
     * Pass a number of bytes to the messaging object. This bytes may represent 0, 1 or more messages
     * \param data the bytes
     * \param size number of bytes
     * \return true if it could take all the data. 
     */
  bool ingest(unsigned char *data, unsigned int size);
  /**
     * \return true if there are messages ready
     */
  bool hasMessages();

  /**
     * Extracts the oldest camera notification from the buffer. It translates it and returns an AiRT message
     * \param buffer where to write the message
     * \param max size of buffer (maximum numbers of bytes to write)
     * \param size [out] returns the size of the message
     * \warning If the provide buffer is smaller than the message, the function fails
     */
  bool popMessage(unsigned char *buffer, const unsigned int max, unsigned int &size);

  // Send the command through the serial port: no payload
  bool sendCommand(Serial &port, StdMessage::RCamCommandType c);
  // Send a command to poweron/off the wifi in the camera
  bool sendWifiOnOffCommand(Serial &port, bool on);
  // Send a command to emulate a key press in the camera (there is no documenation as which number is each key)
  bool sendEmulateKeyCommand(Serial &port, uint8_t key);
  // Send a command to request a configuration parameter
  bool sendGetConfigCommand(Serial &port, StdMessage::RCamConfigParameter setting);
  // Send a command to request a configuration parameter
  bool sendGetXConfigCommand(Serial &port, StdMessage::RCamConfigParameter setting);
  // Send a command for setting a choice config parameter (i.e. ISO, iris, focus...)
  bool sendSetConfigChoice(Serial &port, StdMessage::RCamConfigParameter setting, uint8_t value);
  // Send a command for setting a range config parameter (i.e. MF drive...)
  bool sendSetConfigRange(Serial &port, StdMessage::RCamConfigParameter setting, const uint8_t *value, size_t size);

  /**
   * \return number of discarded bytes so far
   */
  unsigned int discardedBytes() { return discarded_bytes; }

private:
  constexpr static size_t bufferCapacity = 1024;

  boost::circular_buffer<unsigned char> inbuffer;
  enum class Status
  {
    FIRST_SYNC,
    SECOND_SYNC,
    LENGTH,
    CMD,
    OK,
    PAYLOAD,
    DONE
  };
  Status state;
  unsigned int discarded_bytes, payload_read_bytes;
  boost::circular_buffer<unsigned char> config_requests;

  // Parses the buffer and returns true if it finds a message
  bool parse();
  void restart();
  void translateNotification(unsigned char *buffer, const unsigned int max, unsigned int &size);
  void translateConfigNotification(unsigned char *buffer, const unsigned int max, unsigned int &size);
  // Send the command through the serial port: no payload
  bool sendZCAMCommand(Serial &port, unsigned char c);
  // Send the command through the serial port: payload
  bool sendZCAMCommand(Serial &port, unsigned char c, unsigned char *payload, unsigned int size);
  std::vector<unsigned char> message;

#ifdef TESTING
#include "../../external-libs/googletest/include/gtest/gtest_prod.h"
  FRIEND_TEST(ZCamE1MessageProcessor, Overflow);
  FRIEND_TEST(ZCamE1MessageProcessor, ConfigurationRangeNotification);
  FRIEND_TEST(ZCamE1MessageProcessor, ConfigurationChoiceNotification);
#endif
};
};