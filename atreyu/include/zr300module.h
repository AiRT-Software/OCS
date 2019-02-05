#pragma once

#include <string>
#include <stdMessage.h>
#include "internalSource.h"

#include "grabber.h"
#include "filter.h"

namespace airt
{
class Context;

class ZR300Source : public InternalSource
{
  public:

#include "interface/zr300module.h"

    ZR300Source(const std::string &portname,
                std::shared_ptr<Context> context);

    bool dataReady() override;
    bool sendMessage() override;
    bool startDevice() override;
    void stopDevice() override;
    void onMessage(const Message &m) override;

    /**
     * @brief setIMUListener sets an especific function as listener to be executed.
     * This is used for attaching Server::imuListener for receiving the accelerometer
     * angles in the Atreyu server without passing messages.
     * @param listener
     */
    void setIMUListener(std::function<void (double &, float & , float &, float &)> listener)
    {
        externalIMUlistener = listener;
    }

    /**
     * @brief resetIMUListener sets the externalIMUlistener to a function that does nothing
     * with the parameters.
     */
    void resetIMUListener()
    {
        externalIMUlistener = [](double &, float &, float &, float &) {};
    }


  private:
    ZR300Source(const ZR300Source &other) = delete;
    ZR300Source &operator=(ZR300Source other) = delete;

    /**
     * @brief externalIMUlistener is a pointer to the function to be executed inside the motion_callback.
     * Commonly in Server class you attach the Server::imuListener function to this pointer.
     */
    std::function<void (double &, float &, float &, float &)> externalIMUlistener
        = [](double &, float &, float &, float &) {};

    /**
     * @brief motion_callback is a function of this class that will be executed when a new zr300
     * imu data has arrived from the driver.
     * @param entry is the imu (acclerometer or gyroscope) data that has arrived from the zr300 driver.
     */
    void motion_callback(rs::motion_data entry);

    //Pcl-Realsense Grabber
    Grabber grabber;
    size_t maxPointsToSend;
    size_t currentFrame;

    //filtering
    Filter filter;

    std::ofstream acceleratorLogger;
};
};
