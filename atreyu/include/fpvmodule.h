#pragma once

#include "module.h"
#include "grabberD415.h"

#include <stdMessage.h>


namespace airt
{
class Context;

class FPVModule : public Module
{
  public:

#include "interface/fpvmodule.h"

    FPVModule(const std::string &cmdportname, const std::string &pubportname,
                std::shared_ptr<Context> context);

    bool dataReady() override;
    bool sendMessage() override;
    bool startDevice() override;
    void stopDevice() override;
    bool quitDevice() override;
    void onMessage(const Message &m) override;

    bool init();
    bool startStreaming();
    bool stopStreaming();

  private:
    FPVModule(const FPVModule &other) = delete;
    FPVModule &operator=(FPVModule other) = delete;

    GrabberD415 grabber;
    size_t currentFrame;
    std::unique_ptr<unsigned char []> image_jpeg;
    size_t size_image_jpeg;
    size_t max_jpeg_size;

    std::chrono::time_point<std::chrono::steady_clock> lastFrameTS;
    bool fpv_module_is_streaming;
};
};
