#pragma once

#include <string>
#include <stdMessage.h>
#include <module.h>

#include "grabberD415.h"
#include "filter.h"

#include <glm/mat4x4.hpp>


namespace airt
{
class Context;

class D415Module : public Module
{
  public:

#include "interface/d415module.h"

    D415Module(const std::string &cmdportname,const std::string &pubportname, const std::string &subname,
               std::shared_ptr<Context> context);
    ~D415Module();

    bool dataReady() override;
    bool sendMessage() override;
    bool startDevice() override;
    void stopDevice() override;
    bool quitDevice() override;
    void onMessage(const Message &inmsg) override;

    void configureSubscriptions(const std::string &mainpublisher) override;
    void onNotification(const Message &m) override;

  private:
    D415Module(const D415Module &other) = delete;
    D415Module &operator=(D415Module other) = delete;

    bool init();
    bool startCapturing();
    bool stopCapturing();
    void generatePcl(RS_PointRGB * points_rgb, const size_t num_points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud);

    //Pcl-Realsense Grabber
    GrabberD415 grabber;
    size_t currentFrame;

    size_t max_num_points;
    size_t num_points_captured;
    std::unique_ptr<RS_PointRGB []> points_rgb;
    std::chrono::time_point<std::chrono::steady_clock> lastPclFrameTS;
    bool is_capturing;

    //filtering
    Filter filter;

    //transforming
    glm::vec4 currentPositionIPS;                                          // current position coming from the IPS
    float pitchIPS, yawIPS, rollIPS;                                       // current orientation coming from the IPS
    double timestampIPS;                                                   // time stamp of the last position from the IPS
    long numReceivedIPSFrames;

    // mapping cam params
    glm::vec4 cameraOffset;
};
};
