#pragma once

#include <glm/vec3.hpp>

namespace airt
{

/**
 * Given the pitch and yaw, as defined in the atreyu/include/interface/drone.h to a unit vector
 * If pitch = yaw = 0, it returns 1
 * \param pitch in radians
 * \param yaw in radians
 * \return a unit vector pointing in the given direction
 */
glm::vec3 pitchYawToVector(float pitch, float yaw);


/**
 * Given a vector in pozyx coordinate system, as defined in atreyu/include/interface/drone.h, returns the pitch, yaw and roll angles
 * \param v the vector
 * \param pitch [out] The angle between the vector and the XY plane
 * \param yaw [out] The angle between the vector and the Y directio
 * 
 */

void vectorToPitchYaw(const glm::vec3 &v, float &pitch, float &yaw);
}; // namespace airt