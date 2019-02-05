#pragma once

#include <memory>
#include <cstddef>
#include <zmqpp.hpp>

namespace airt
{
class Message
{
public:
  Message();
  Message(Message &&o);
  ~Message();
  template <typename Type>
  void add_raw(Type *part, size_t const data_size)
  {
    msg->add_raw<Type>(part, data_size);
  }
  template <typename Type>
  Type get(const size_t part) const
  {
    return msg->get<Type>(part);
  }

  size_t size(const size_t part) const;
  size_t parts() const { return msg->parts(); };

  Message copy();

private:
  Message(const Message &) = delete;
  Message &operator=(const Message &) = delete;
  std::unique_ptr<zmqpp::message> msg;
  friend class Socket;
};
};