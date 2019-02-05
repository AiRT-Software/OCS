
#include <glm/geometric.hpp>
#include <math.h>

#include "missionContainer.h"
#include "log.h"
#include "utils.h"
#include "glmutils.h"


using airt::Log;
using MissionContainer::GimbalParameters;
using MissionContainer::Map;
using MissionContainer::RecCamParameters;
using MissionContainer::WayPoint;

const WayPoint &Map::getWaypoint(uint32_t planIndex, uint32_t wpId)
{
    return paths[planIndex].waypoints[wpId];
}

int Map::findRecCamParameters(uint32_t planIndex, uint32_t wpId)
{
    auto &camParams = paths[planIndex].rcamParams;
    auto it = airt::binary_find(camParams.begin(), camParams.end(), RecCamParameters{wpId, {}},
                                [](const RecCamParameters &lhs, const RecCamParameters &rhs) {
                                    return lhs.wpId < rhs.wpId;
                                });

    if (it != camParams.end())
    {
        int index = it - camParams.begin();
        return index;
    }
    else
    {
        // There is nothing to do for this wp
        return -1;
    }
}

const RecCamParameters &Map::getRecCamParameters(uint32_t planIndex, uint32_t rcamIdx)
{
    if (rcamIdx < 0 || rcamIdx >= paths[planIndex].rcamParams.size())
    {
        Log::error("Wrong Rcam index: {}", rcamIdx);
        rcamIdx = 0;
    }
    return paths[planIndex].rcamParams[rcamIdx];
}

int Map::findGimbalParameters(uint32_t planIndex, uint32_t wpId)
{
    auto &gimbalParams = paths[planIndex].gimbalParams;
    auto it = airt::binary_find(gimbalParams.begin(), gimbalParams.end(), GimbalParameters{wpId, 0.0f, 0.0f, 0.0f, 0},
                                [](const GimbalParameters &lhs, const GimbalParameters &rhs) {
                                    return lhs.wpId < rhs.wpId;
                                });

    if (it != gimbalParams.end())
    {
        int index = it - gimbalParams.begin();
        return index;
    }
    else
    {
        // There is nothing to do for this wp
        return -1;
    }
}

const GimbalParameters &Map::getGimbalParameters(uint32_t planIndex, uint32_t gimbalIdx)
{
    if (gimbalIdx < 0 || gimbalIdx >= paths[planIndex].gimbalParams.size())
    {
        Log::error("Wrong gimbal index: {}. Returning the first one", gimbalIdx);
        gimbalIdx = 0;
    }
    return paths[planIndex].gimbalParams[gimbalIdx];
}

#define REPEAT -9876543.2f

void Map::effectiveGimbalPosition(uint32_t planIndex, uint32_t gimbalIdx, uint32_t waypointIdx, float &pitch, float &roll, float &yaw)
{
    const auto &gc = paths[planIndex].gimbalParams[gimbalIdx];
    const auto &wp = paths[planIndex].waypoints[waypointIdx];

    switch (gc.mode)
    {
    case LOOK_AT_POI: // always looks at a given coordinate
    {
        glm::vec3 from(wp.x, wp.y, wp.z), to(gc.x_pitch, gc.y_roll, gc.z_yaw);
        airt::vectorToPitchYaw(to - from, pitch, yaw);
        roll = 0.0f;
    }
    break;
    case LOOK_AHEAD:           // follows the tangent of the trajectory
    case LOOK_AHEAD_FIX_PITCH: // follows the tangent of the trajectory, but the pitch is fixed at the given value
    {
        if (waypointIdx + 1 < paths[planIndex].waypoints.size())
        {
            // Next waypoint exists
            const auto &nwp = paths[planIndex].waypoints[waypointIdx + 1];
            glm::vec3 from(wp.x, wp.y, wp.z), to(nwp.x, nwp.y, nwp.z);
            airt::vectorToPitchYaw(to - from, pitch, yaw);
            roll = 0.0f;
        }
        else
        {
            pitch = REPEAT;
            yaw = REPEAT;
        }

        if (gc.mode == LOOK_AHEAD_FIX_PITCH)
        {
            pitch = gc.x_pitch;
        }
    }
    break;
    case BLOCK_DIRECTION:
        pitch = gc.x_pitch;
        roll = gc.y_roll;
        yaw = gc.z_yaw;
        break;
    }
}

std::vector<MissionContainer::GimbalWaypoint> Map::compileGimbalFlightPlan(uint32_t planIndex)
{
    std::vector<MissionContainer::GimbalWaypoint> result;
    result.reserve(paths[planIndex].waypoints.size());

    const std::vector<GimbalParameters> &gplan = paths[planIndex].gimbalParams;
    const std::vector<WayPoint> &wpplan = paths[planIndex].waypoints;

    size_t gimbalIdx = 0;
    for (size_t i = 0; i < wpplan.size(); i++)
    {
        if (gimbalIdx + 1 < gplan.size() && gplan[gimbalIdx + 1].wpId == i)
        {
            // This waypoint has a specific configuration
            gimbalIdx++;
        }

        MissionContainer::GimbalWaypoint gimbal;
        effectiveGimbalPosition(planIndex, gimbalIdx, i, gimbal.pitch, gimbal.roll, gimbal.yaw);
        if (gimbal.pitch == REPEAT)
        {
            if (i > 0)
                gimbal.pitch = result[i - 1].pitch;
            else
                gimbal.pitch = 0.0f;
        }
        if (gimbal.yaw == REPEAT)
        {
            if (i > 0)
                gimbal.yaw = result[i - 1].yaw;
            else
                gimbal.yaw = 0.0f;
        }

        result.push_back(gimbal);
    }

    // Compute speed
    for (size_t i = 0; i < result.size() - 1; i++)
    {
        // Compute how long it will take the drone to go from wp i to wp i+1
        const auto &wp = wpplan[i];
        const auto &nwp = wpplan[i + 1];
        float d_meters = glm::distance(glm::vec3(wp.x, wp.y, wp.z), glm::vec3(nwp.x, nwp.y, nwp.z)) / 1000.0f;
        float t = d_meters / wp.speed;

        // Maximum change in a gimbal angle:
        float maxdelta = std::max(
            fabs(result[i].pitch - result[i + 1].pitch),
            std::max(
                fabs(result[i].roll - result[i + 1].roll),
                fabs(result[i].yaw - result[i + 1].yaw)));
        result[i].speed = maxdelta/t;
    }
    result.back().speed = 0;
    return result;
}