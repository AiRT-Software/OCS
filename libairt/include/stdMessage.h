#pragma once

#include "net/message.h"
#include "net/socket.h"

namespace airt
{

constexpr uint8_t AIRT_SIGNATURE = 'A';

struct AIRT_Message_Header
{
  const uint8_t airt; // 'A'
  const uint8_t module;
  const uint8_t action;
  AIRT_Message_Header(uint8_t module, uint8_t action) : airt(AIRT_SIGNATURE), module(module), action(action){};
  bool operator==(const struct AIRT_Message_Header &other) const
  {
    return airt == other.airt && module == other.module && action == other.action;
  }
};

struct StdMessage : public AIRT_Message_Header
{
#include "messages/MessageCodes.h"

  StdMessage(Modules module, uint8_t action) : AIRT_Message_Header(module, action){};
  StdMessage(uint8_t module, uint8_t action) : AIRT_Message_Header(static_cast<enum Modules>(module), action){};

  bool operator==(const struct StdMessage &other) const
  {
    return AIRT_Message_Header::operator==(other);
  }

  Message toMsg()
  {
    Message msg;
    msg.add_raw(this, sizeof(StdMessage));
    return msg;
  }
};

struct StdNotification : public StdMessage
{
  explicit StdNotification(uint8_t action) : StdMessage(STD_NOTIFICATIONS_MODULE, action) {}
};

struct StdCommand : public StdMessage
{
  explicit StdCommand(uint8_t action) : StdMessage(STD_COMMANDS_MODULE, action) {}
};

struct MsgToOtherModule : public StdMessage
{
  explicit MsgToOtherModule(uint8_t from) : StdMessage(MESSAGE_BUS_MODULE, from) {}
};

std::string to_string(const Message &m);
std::string to_string(const StdMessage &m);

inline bool isAIRTMessage(const StdMessage &m)
{
  return m.airt == AIRT_SIGNATURE;
}

inline bool isAIRTMessage(const uint8_t *m, size_t size)
{
  return size >= 3 && m[0] == AIRT_SIGNATURE;
}

inline bool isAIRTMessage(const Message &m)
{
  return isAIRTMessage(m.get<const uint8_t *>(0), m.size(0));
}

inline bool isCommand(const StdMessage &m)
{
  return (m.module & 128) == 0 && m.airt == AIRT_SIGNATURE;
}

inline bool isStdCommand(const StdMessage &m)
{
  return m.module == StdMessage::STD_COMMANDS_MODULE && m.airt == AIRT_SIGNATURE;
}

inline bool isStdCommand(const uint8_t *m, size_t size)
{
  return size >= 3 && m[1] == StdMessage::STD_COMMANDS_MODULE && m[0] == AIRT_SIGNATURE;
}

inline bool isStdCommand(const Message &m)
{
  return isStdCommand(m.get<const uint8_t *>(0), m.size(0));
}

inline bool isNotification(const StdMessage &m)
{
  return (m.module & 128) != 0 && m.airt == AIRT_SIGNATURE;
}

inline bool isStdNotification(const StdMessage &m)
{
  return m.module == StdMessage::STD_NOTIFICATIONS_MODULE && m.airt == AIRT_SIGNATURE;
}

inline bool isStdNotification(const uint8_t *m, size_t size)
{
  return size >= 3 && m[1] == StdMessage::STD_NOTIFICATIONS_MODULE && m[0] == AIRT_SIGNATURE;
}
inline bool isStdNotification(const Message &m)
{
  return isStdNotification(m.get<const uint8_t *>(0), m.size(0));
}

inline static uint8_t getMessageSignature(const Message &msg)
{
  return *(msg.get<const uint8_t *>(0));
}

inline static uint8_t getMessageModule(const Message &msg)
{
  //  return ((msg.get<const uint8_t *>(0))[1]) & 0x7F;
  return ((msg.get<const uint8_t *>(0))[1]);
}

inline static uint8_t getMessageAction(const Message &msg)
{
  return (msg.get<const uint8_t *>(0))[2];
}

inline void sendNotification(Socket &socket, StdMessage::NotificationType notification)
{
  auto msg = StdNotification(notification).toMsg();
  socket.send(msg);
}
template <typename N>
void sendOnePartMessage(Socket &socket, const N *m) {
  Message out;
  out.add_raw(m, sizeof(N));
  socket.send(out);
} 

inline void sendOnePartMessage(Socket &socket, const void *m, size_t size) {
  Message out;
  out.add_raw(m, size);
  socket.send(out);
} 

template <typename N, typename M>
void sendTwoPartMessage(Socket &socket, const N *part1, const M *part2) {
  Message out;
  out.add_raw(part1, sizeof(N));
  out.add_raw(part2, sizeof(M));
  socket.send(out);
} 

template <typename N>
void sendTwoPartMessage(Socket &socket, const N *m, const void *data, size_t size) {
  Message out;
  out.add_raw(m, sizeof(N));
  out.add_raw(data, size);
  socket.send(out);
} 

template <typename N>
void sendTwoPartMessage(Socket &socket, const N *m, const std::string &data) {
  Message out;
  out.add_raw(m, sizeof(N));
  out.add_raw(data.c_str(), data.size() + 1); // adds one for the z in asciiz
  socket.send(out);
} 

template <typename N>
void sendThreePartMessage(Socket &socket, const N *data1, const std::string &data2, const std::string &data3) {
  Message out;
  out.add_raw(data1, sizeof(N));
  out.add_raw(data2.c_str(), data2.size() + 1); // adds one for the z in asciiz
  out.add_raw(data3.c_str(), data3.size() + 1); // adds one for the z in asciiz
  socket.send(out);
} 

template <typename N, typename M>
void sendThreePartMessage(Socket &socket, const N *data1, const M *data2, const std::string &data3) {
  Message out;
  out.add_raw(data1, sizeof(N));
  out.add_raw(data2, sizeof(M)); 
  out.add_raw(data3.c_str(), data3.size() + 1); // adds one for the z in asciiz
  socket.send(out);
} 


template <typename N>
void sendThreePartMessage(Socket &socket, const N *data1, const std::string &data2, const void *data3, size_t size3) {
  Message out;
  out.add_raw(data1, sizeof(N));
  out.add_raw(data2.c_str(), data2.size() + 1); // adds one for the z in asciiz
  out.add_raw(data3, size3); // adds one for the z in asciiz
  socket.send(out);
} 

template <typename N>
void sendNotification(Socket &socket, StdMessage::Modules module, N notification)
{
  auto msg = StdMessage(module, static_cast<uint8_t>(notification)).toMsg();
  socket.send(msg);
}

inline void sendCommand(Socket &socket, StdMessage::CommandType command)
{
  auto msg = StdCommand(command).toMsg();
  socket.send(msg);
}

template <typename N>
void sendCommand(Socket &socket, StdMessage::Modules module, N command)
{
  auto msg = StdMessage(module, static_cast<uint8_t>(command)).toMsg();
  socket.send(msg);
}

// If you change the following pathname, update it also in trunk/release-files/scripts/postinst
static const std::string installUpdatesPathname("/usr/share/airt/install/airt.deb");
};
