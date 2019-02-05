#pragma once

#include <string>
#include <stdMessage.h>
#include "module.h"

namespace airt
{
class Context;
class PointCloudDatabase;
class PointCloud;

class MapperModule : public Module
{
public:
#include "interface/mappermodule.h"

  MapperModule(const std::string &portname, const std::string &pubname, const std::string &subname,
               std::shared_ptr<Context> context);
  ~MapperModule();
  void onMessage(const Message &m) override;
  bool dataReady() override { return false; }
  bool sendMessage() override { return false; }
  void configureSubscriptions(const std::string &mainpublisher) override;
  void onNotification(const Message &m) override;
  void onIdle() override;

private:
  void reset();

  std::string guid;
  std::unique_ptr<PointCloudDatabase> ddbb;
  std::deque<std::shared_ptr<PointCloud>> publishingQueue;

  // Maximum number of clouds to be published in one cycle (for not stalling the thread when someone requests all clouds)
  static constexpr size_t maxNumberCloudsPublishedPerCycle = 3;
  enum class State
  {
    NonInitialized,
    Idle,
    Ready,
    Mapping,
    Paused
  };

  State state;
  void publishPointCloud(const glm::vec3 &pos, float roll, float pitch, float yaw, size_t numPoints, const void *points);
  bool checkDDBB();
};
}; // namespace airt
