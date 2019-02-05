#include "glmutils.h"

#include <glm/geometric.hpp>

glm::vec3 airt::pitchYawToVector(float pitch, float yaw)
{
  auto cp = cosf(pitch);
  glm::vec3 res{
      sin(-yaw) * cp,
      cos(-yaw) * cp,
      sin(pitch)};
  return glm::normalize(res);
}

void airt::vectorToPitchYaw(const glm::vec3 &v, float &pitch, float &yaw)
{
  if (v.x == 0.0f && v.y == 0.0f) {
    yaw = 0.0f;
  } else {
    yaw =  atan2(v.y, v.x) - M_PI_2;
  }

  float r = glm::length(v);
  if (r == 0.0f) {
    pitch = 0.0f;
  } else {
    pitch = M_PI/2.0 - acos(v.z / r);
  }
}
