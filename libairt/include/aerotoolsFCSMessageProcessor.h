#pragma once

#include <boost/circular_buffer.hpp>

namespace airt
{

class Serial;
enum FCSMultiplexerCommandType : unsigned char;
/**
 * \class AeroToolsFCSMessageProcessor
 * This class is in charge of translating between AiRT and Aerotools multiplexer protocols.
 * There are two kinds of messages:
 * -Commands: sent from AiRT to Aerotools multiplexer
 * -Notifications: sent from Aerotools multiplexer to AiRT
 * 
 * In order to add more commands:
 * 
 * 1. Add a new constant to the FCSMultiplexerCommandType in MessageCodes.cs, and the 
 * corresponding command code to the Commands enum in the cpp file
 * 2. If the command has no payload, add a new case to AeroToolsFCSMessageProcessor::sendCommand
 * 3. If the command has payload, add a new method called sendXXXCommand, where XXX defines 
 * the type of command and it has the required parameters bo build the Aerotools message.
 * 
 * In order to add more notifications:
 * 1. Add a new constant to the Notificatios enum in the cpp file
 * 2. Add a new case to the switch in the AeroToolsFCSMessageProcessor::translateNotification method to
 * translate the message into the AiRT notification
*/

class AeroToolsFCSMessageProcessor
{
public:
  AeroToolsFCSMessageProcessor();

  void setGPSOrigin(float longitude, float latitude) {
    pozyx2gps_origin_lon = longitude;
    pozyx2gps_origin_lat = latitude;
  }
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
  bool sendCommand(Serial &port, enum FCSMultiplexerCommandType c);

  /**
   *  Send the drone a command to fly to a certain location 
   * \param x x coordinate in poxyz space (mm)
   * \param y y coordinate in pozyx space (mm)
   * \param height the height, in mm
   * */
  bool sendGoToCommand(Serial &port, float x, float y, float height);
  /**
   *  Upload a mission waypoint  
   * \param id the waypoint id
   * \param x x coordinate in poxyz space (mm)
   * \param y y coordinate in pozyx space (mm)
   * \param height the height, in mm
   * */
  bool sendAddWaypointCommand(Serial &port, uint16_t id, float x, float y, float height);

  /**
   * Send take off command
   * \param height_mm the height to fly after takeoff (in m)
   * */
  bool sendTakeOffCommand(Serial &port, float height_m);

  /**
   * 
   * Send set mode command 
   * \param mode see FCSFlightModes
   */
  bool sendSetModeCommand(Serial &port, uint8_t mode);

  /**
   * Send set speed command 
   * \param speed in m/s
   */
  bool sendSetSpeed(Serial &port, uint8_t speed);

  /**
   * Send the create mission command
   * \param numWPs number of waypoints in the mission
   * */
  bool sendCreateMission(Serial &port, uint16_t numWPs);

private:
  constexpr static size_t bufferCapacity = 1024;
  float pozyx2gps_origin_lon, pozyx2gps_origin_lat;

  void pozyx2gps(float x_mm, float y_mm, float z_mm, int32_t &lon_deg, int32_t &lat_deg, int32_t &h_mm);
  void gps2pozyx(int32_t lon_deg, int32_t lat_deg, int32_t h_mm, float &x_mm, float &y_mm, float &z_mm);

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
  bool sendATCommand(Serial &port, unsigned char c, const void *payload, unsigned int size);
  std::vector<unsigned char> message;

#ifdef TESTING
#include "../../external-libs/googletest/include/gtest/gtest_prod.h"
  FRIEND_TEST(AeroToolsFCSMessageProcessor, Overflow);
#endif
};
};