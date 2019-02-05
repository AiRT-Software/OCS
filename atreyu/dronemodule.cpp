#include "dronemodule.h"
#include <stdMessage.h>
#include <iomanip>
#include <log.h>
#include "globalSettings.h"

namespace airt
{
#include "interface/pozyxmodule.h"
#include "interface/fcsmodule.h"
#include "interface/gimbalmodule.h"
}; // namespace airt

using airt::DroneModule;

DroneModule::DroneModule(const std::string &portname, const std::string &pubname, const std::string &subname,
                         std::shared_ptr<Context> context)
    : Module(portname, pubname, subname, context)
{
    assert(context);

    // if (!GlobalSettings::getValue("plan_exec_preflight_check_timeout_seconds", preflightCheckTimeout))
    // {
    //     preflightCheckTimeout = 3.0f;
    //     Log::warn("Time out for preflight checks not defined (plan_exec_preflight_check_timeout_seconds). Set to {} s", preflightCheckTimeout);
    // }

    Log::info("Drone module initialized");
}

void DroneModule::configureSubscriptions(const std::string &mainpublisher)
{
    assert(subsocket);
    subsocket->connect(mainpublisher);
    uint8_t fcssignature[]{'A', StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE};
    subsocket->subscribe(fcssignature, airt::arraySize(fcssignature));
    uint8_t pozyxsignature[]{'A', StdMessage::POSITIONING_NOTIFICATIONS_MODULE};
    subsocket->subscribe(pozyxsignature, airt::arraySize(pozyxsignature));
    uint8_t gimbalsignature[]{'A', StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE};
    subsocket->subscribe(gimbalsignature, airt::arraySize(gimbalsignature));
}

void DroneModule::onNotification(const Message &m)
{

    uint8_t module = airt::getMessageModule(m);
    uint8_t action = airt::getMessageAction(m);
    Log::info("DRONE MODULE: Notification from {}. Action: {}", module, action);

    switch (module)
    {
    case StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE:
        if (action == StdMessage::FCS_ROLLPITCHYAW_NOTIFICATION)
        {
            const auto rpy = m.get<const RollPitchYawNotification *>(0);
            drone.updatePitchRoll(rpy->pitch, rpy->yaw, airt::msSinceEpoch());
            publishPose();
        }
        break;
    case StdMessage::POSITIONING_NOTIFICATIONS_MODULE:
        if (action == StdMessage::IPS_DATA)
        {
            const auto pp = m.get<const PositioningFrame *>(0);
            drone.updatePosYaw(glm::vec3(pp->x, pp->y, pp->z), pp->yaw, pp->timestamp);
            publishPose();
        }
        break;
    case StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE:
        if (action == StdMessage::GIMBAL_ANGLE_CHANGED_NOTIFICATION)
        {
            const auto gn = m.get<const GimbalAngleNotification *>(0);
            drone.updateGimbalAngles(gn->pitch, gn->yaw, gn->roll);
            publishGimbalPose();
        }
        else if (action == StdMessage::GIMBAL_GOTO_ANGLE_NOTIFICATION)
        {
            const auto gp = m.get<const GotoAngleNotification *>(0);
            drone.updateGimbalAngles(gp->pitch, gp->yaw, gp->roll);
            publishGimbalPose();
        }
        break;
    default:
        Log::critical("Error: message from unexpected module");
    }
}

void DroneModule::publishPose()
{
    struct DronePositionPose dpp;
    auto pos = drone.getPos();
    dpp.x = pos.x;
    dpp.y = pos.y;
    dpp.z = pos.z;
    drone.getDroneAngles(dpp.pitch, dpp.roll, dpp.yaw);
    dpp.timestamp = airt::msSinceEpoch();
    airt::sendOnePartMessage(*pubsocket, &dpp);
}

void DroneModule::publishGimbalPose()
{
    struct DroneGimbalPose gp;
    drone.getGimbalAngles(gp.pitch, gp.roll, gp.yaw);
    gp.timestamp = airt::msSinceEpoch();
    airt::sendOnePartMessage(*pubsocket, &gp);
}

void DroneModule::onMessage(const Message &inmsg)
{
    uint8_t command = airt::getMessageAction(inmsg);
    switch (command)
    {
    case StdMessage::DRONE_SET_ZCAMERA_OFFSET:
    {
        const auto zo = inmsg.get<const SetZCameraOffset *>(0);
        std::vector<double> offset{zo->offsetX, zo->offsetY, zo->offsetZ};
        GlobalSettings::setArrayValues("drone_zcam_offset", offset);
        GlobalSettings::save();
    }
        // FALL THROUGH
    case StdMessage::DRONE_GET_ZCAMERA_OFFSET:
    {
        std::vector<double> offset;
        DroneZCamOffsetNotification zon;
        if (!GlobalSettings::getArrayValues("drone_zcam_offset", offset))
        {
            Log::error("The offset of the Z-camera is requested but it has not been set before. Using 0, 0, 0");
            zon.offsetX = zon.offsetY = zon.offsetZ = 0.0f;
        }
        else
        {
            zon.offsetX = offset[0];
            zon.offsetY = offset[1];
            zon.offsetZ = offset[2];
        }
        sendOnePartMessage(*pubsocket, &zon);
        break;
    }
    }
    airt::sendNotification(*pubsocket, StdMessage::DRONE_NOTIFICATIONS_MODULE, StdMessage::ACK);
}
