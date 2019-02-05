#pragma once

#include <string>
#include <stdMessage.h>
#include <pcl/io/pcd_io.h>

#include "module.h"


namespace airt
{
class Context;


class PCLFilePlayer : public Module
{
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointNormalT;

public:
  PCLFilePlayer(const std::string &cmdportname, const std::string &pubportname,
                std::shared_ptr<Context> context);

  void setBaseName(const std::string &basename)
  {
    pclFilename = basename;
  }
  void setReplayFrequency(float freq) {
    replayFreq = freq;
  }

  bool dataReady() override;
  bool sendMessage() override;
  bool startDevice() override;
  void stopDevice() override;

private:
  bool readFile(const std::string &basename, size_t frame);

  PCLFilePlayer(const PCLFilePlayer &other) = delete;
  PCLFilePlayer &operator=(PCLFilePlayer other) = delete;
  std::string pclFilename;
  float replayFreq;
  bool publishing;
  size_t currentFrame;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud;
  /**
   * @brief maxPointsToSend limits the number of points to be sent in a message to the clients.
   * The pointcloud will be split if necessary in the sendMessage method.
   */
  size_t maxPointsToSend;
  size_t totalFramesToPublish;
};
};
