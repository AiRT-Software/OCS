#pragma once

#include <boost/circular_buffer.hpp>
#include <vector>

namespace airt
{

class Serial;
enum GimbalMultiplexerCommandType : unsigned char;
/**
 * \class AeroToolsGimbalMessageProcessor
 * This class is in charge of translating between AiRT and Aerotools gimbal multiplexer protocols.
 * There are two kinds of messages:
 * -Commands: sent from AiRT to Aerotools multiplexer
 * -Notifications: sent from Aerotools multiplexer to AiRT
 * 
 * In order to add more commands:
 * 
 * 1. Add a new constant to the GimbalMultiplexerCommandType in MessageCodes.cs, and the 
 * corresponding command code to the Commands enum in the cpp file
 * 2. If the command has no payload, add a new case to AeroToolsGimbalMessageProcessor::sendCommand
 * 3. If the command has payload, add a new method called sendXXXCommand, where XXX defines 
 * the type of command and it has the required parameters bo build the Aerotools message.
 * 
 * In order to add more notifications:
 * 1. Add a new constant to the Notificatios enum in the cpp file
 * 2. Add a new case to the switch in the AeroToolsGimbalMessageProcessor::translateNotification method to
 * translate the message into the AiRT notification
*/

class AeroToolsGimbalMessageProcessor
{
public:
  AeroToolsGimbalMessageProcessor();

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
     * Extracts the oldest FCS notification from the buffer. It translates it and returns an AiRT message
     * \param buffer where to write the message
     * \param max size of buffer (maximum numbers of bytes to write)
     * \param size [out] returns the size of the message
     * \warning If the provide buffer is smaller than the message, the function fails
     */
  bool popMessage(unsigned char *buffer, const unsigned int max, unsigned int &size);

  /**
   * \return number of discarded bytes so far
   */
  unsigned int discardedBytes() { return discarded_bytes; }

  // Send the command through the serial port: no payload
  bool sendCommand(Serial &port, enum GimbalMultiplexerCommandType c);

  /**
   *  Send the command to goto to a certain angle wrt to the zero position
   * \param port the Serial port to use
   * \param pitch the pitch angle, in radians
   * \param roll the roll angle, in radians
   * \param yaw the yaw angle, in radians
   * \param speedRadPerSec speed, in radians per second
   * \return true if could send the command 
   */
  bool sendGotoAngle(Serial &port, float pitch, float roll, float yaw, float speedRadPerSec);

  bool sendMovePitch(Serial &port, float speed);
  bool sendMoveRoll(Serial &port, float speed);
  bool sendMoveYaw(Serial &port, float speed);
  
  // Send the command to set the speed 

private:
  constexpr static size_t bufferCapacity = 1024;
  /**
     * Computes the checksum of the message
     */
  unsigned char checksum(unsigned char *buffer);

  boost::circular_buffer<unsigned char> inbuffer;
  enum class Status
  {
    FIRST_SYNC,
    SECOND_SYNC,
    ID,
    LENGTH,
    PAYLOAD,
    CHKSUM,
    DONE
  };
  Status state;
  unsigned int discarded_bytes, payload_read_bytes;

  // Parses the buffer and returns true if it finds a message
  bool parse();
  void restart();
  void translateNotification(unsigned char *buffer, const unsigned int max, unsigned int &size);
  // Send the command through the serial port: no payload
  bool sendATCommand(Serial &port, unsigned char c);
  // Send the command through the serial port: payload
  bool sendATCommand(Serial &port, unsigned char c, unsigned char *payload, unsigned int size);
  std::vector<unsigned char> message;

#ifdef TESTING
#include "../../external-libs/googletest/include/gtest/gtest_prod.h"
  FRIEND_TEST(AeroToolsFCSMessageProcessor, Overflow);
#endif
};
};