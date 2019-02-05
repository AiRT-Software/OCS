#include <memory>
#include <proxy_steerable.hpp>

namespace airt
{
class Socket;

class ProxySteerable
{
  public:
    ProxySteerable(Socket &recv, Socket &send, Socket &ctrl);
    ~ProxySteerable();
  private:
    std::unique_ptr<zmqpp::proxy_steerable> proxy;
};
};
