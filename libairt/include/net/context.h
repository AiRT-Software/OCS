#pragma once

#include <memory>

namespace zmqpp
{
class context;
};

namespace airt
{
class Context
{
  public:
    Context();
    ~Context();
  private:
    std::unique_ptr<zmqpp::context> context;
    friend class Socket;
};
};