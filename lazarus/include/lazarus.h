#pragma once

#include <memory>
#include <array>
#include <net/socket.h>
#include <reliableExternalClient.h>

namespace airt
{

class Lazarus
{
public:
  Lazarus();
  void run();

protected:
  void processNotification(const Message &msg);
  bool processUpdate(const Message &msg);

private:
  // Checks whether there is other lazarus instance running (e.g., we are updating)
  void config();
  bool isAtreyuRunning();
  void launchAtreyu();
  bool installUpdate();
  std::shared_ptr<Context> context;
  Socket subscriber;
  std::unique_ptr<ReliableExternalClient> reqSrv;
  bool done, poweroff;
  std::chrono::time_point<std::chrono::steady_clock> lastMessageTS;
  std::string atreyuProcessName, atreyuExecPath, atreyuArgs, atreyuExeFullname, atreyuWorkingDir;
  std::string installUpdatesDir;
  double timeoutToCheckPID, timeoutToKill;
};
};
