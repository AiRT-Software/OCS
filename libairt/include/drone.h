#pragma once

#include <chrono>
#include <glm/vec3.hpp>

namespace airt
{

/**
     * \class Drone
     * Represents the current status of the drone (position + orientation) and gimbal (position + orientation). It receives 
     * notifications from all the sensors and integrate them into a unified status.
     * It also provides methods for controlling the attached gimbal.
     * 
     **/
class Drone
{
public:
  Drone();
  void updatePos(const glm::vec3 &pos, double timestamp);
  void updateYaw(float yaw);
  void updatePosYaw(const glm::vec3 &pos, float yaw, double timestamp);
  void updatePitchRoll(float pitch, float roll, double timestamp);

  /**
   * Method to obtain the drone orientation
   */
  void getDroneAngles(float &pitch, float &roll, float &yaw) const {
    pitch = this->pitch;
    roll = this->roll;
    yaw = this->yaw;
  }


  /**
   * The gimbal is at the given direction 
   * \param pitch in radians
   * \param yaw in radians
   * \param roll in radians
   */
  void updateGimbalAngles(float pitch, float yaw, float roll);

  /**
   * Return the current gimbal configuration
   * \param roll in radians
   * \param pitch in radians
   * \param yaw in radians
   */
  void getGimbalAngles(float &roll, float &pitch, float &yaw);

  /**
     * \return the current position of the drone
     * */
  glm::vec3 getPos() const { return position; }

  /**
     * Sets the gimbal offset with respect to the center of drone
     * \param offset x, y, z of the center of the gimbal
     */
  void setGimbalOffset(const glm::vec3 &offset);

  /**
     * Sets the position of the recording camera with respect to the center of the drone
     * 
     */
  void setRecCamOffset(const glm::vec3 &offset);

  /** 
   * Sets the position of the depth camera with respect to the center of the drone
   */
  void setZCamOffset(const glm::vec3 &offset);

private:
  // These two come from pozyx
  glm::vec3 position; // Position of the center of the drone
  float yaw;          // Angle with respect to the North (Poxyz's Y axis) Positive to the left, in radians
  double latestPosTS;
  // These two come from the fcs
  float pitch; // Angle around the local X axis of the drone, in radians (negative nose down, positive nose up), in radians
  float roll;  // Angle around the front axis of the drone, in radians (negative head tilted left, positive head tilted right), in radians
  double latestRollPitchTS;
  float pitchSpeed, rollSpeed, yawSpeed;                     // Instantaneous change speed.
  glm::vec3 gimbalOffset;                                    // Gimbal offset from the center of the drone (in mm). This can change, as the gimbal can be installed
                                                             // on the top or at the bottom of the drone
  float gimbalPitch, gimbalRoll, gimbalYaw;                  // Current gimbal pose
};
}; // namespace airt