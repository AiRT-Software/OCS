#include <algorithm>
#include "drone.h"

using airt::Drone;

Drone::Drone() : position{0.0f}, yaw{0.f}, pitch{0.0f}, roll{0.0f},
                 pitchSpeed{0.0f}, rollSpeed{0.0f}, yawSpeed{0.0f}, gimbalOffset{0.0f} {};

void Drone::updatePos(const glm::vec3 &pos, double timestamp)
{
    position = pos;
    latestPosTS = timestamp;
}

void Drone::updateYaw(float yaw)
{
    this->yaw = yaw;
}

void Drone::updatePosYaw(const glm::vec3 &pos, float yaw, double timestamp)
{
    position = pos;
    this->yaw = yaw;
    latestPosTS = timestamp;
}

void Drone::updatePitchRoll(float pitch, float roll, double timestamp)
{
    this->pitch = pitch;
    this->roll = roll;
    latestRollPitchTS = timestamp;
}

void Drone::updateGimbalAngles(float pitch, float yaw, float roll)
{
    gimbalPitch = pitch;
    gimbalRoll = roll;
    gimbalYaw = yaw;
}

void Drone::getGimbalAngles(float &roll, float &pitch, float &yaw)
{
    roll = gimbalRoll;
    pitch = gimbalPitch;
    yaw = gimbalYaw;
}

