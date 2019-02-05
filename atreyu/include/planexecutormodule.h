#pragma once

#include <string>
#include <stdMessage.h>
#include <utils.h>
#include <iosfwd>
#include <boost/optional.hpp>
#include <glm/mat4x4.hpp>
#include "module.h"

#include <boost/sml.hpp>

#include <missionLoader.h>
#include <missionContainer.h>

namespace airt
{
class Context;

class PlanExecutorModule : public Module
{
public:
#include "interface/planexecutormodule.h"

  PlanExecutorModule(const std::string &portname, const std::string &pubname, const std::string &subname,
                     std::shared_ptr<Context> context);

  void onMessage(const Message &m) override;
  bool dataReady() override { return false; }
  bool sendMessage() override { return false; }
  void configureSubscriptions(const std::string &mainpublisher) override;
  void onNotification(const Message &m) override;
  void onIdle() override;

private:
  std::string missionBaseName; // mission base name (guid)
  std::string missionFilename; // mission filename
  size_t planIndex;            // which plan inside the mission
  std::string preflightCheckErrorString;
  boost::optional<glm::mat4> mapToPozyx;
  glm::mat4 pozyxToMap;
  glm::vec4 currentPositionIPS;                                          // current position coming from the IPS
  float pitchIPS, yawIPS, rollIPS;                                       // current orientation coming from the IPS
  double timestampIPS;                                                   // time stamp of the last position from the IPS
  std::chrono::time_point<std::chrono::steady_clock> timestampIPSMyTime; // time stamp in my watch
  MissionContainer::Map map;
  uint16_t lastWPReached;
  bool bypassPreflightChecks;
  bool fcsPresent;
  float motorsThrottle;
  float motorsArmPowerThreshold; // minimum power of motors to be considered to be "armed"
  float preflightCheckTimeout;
  float warningDistanceToFirstWPMeters; // If the distance between take off and first wp is farther than this distance, we issue a warning
  float minimumTakeOffHeightMeters; // minimum height for take off (in mm)
  std::vector<MissionContainer::GimbalWaypoint> gimbalFlightPlan;

  void reset();
  void notify(StdMessage::PlanExecutorNotificationType notification);
  void notify(StdMessage::PlanExecutorNotificationType notification, const std::string &data);
  void notifyLoadingPlan();
  void notifyLoadPlanFailed();
  void notifyPlanLoaded();
  void notifyWaypointReached();
  bool loadPlan();
  void preflightChecks();
  void checkFCS();
  void notifyPreflightChecksFailed();
  void arm();
  void fly();
  void land();
  void takeoff();
  void uploadPlanToFCS();
  bool arrivedToLastWaypoint();
  void finishExecution();
  void checkFirstWPCloseness();
  void processWaypointPayload(uint32_t wpId);
  void processRCamCommands(uint32_t wpId);
  void processGimbalCommands(uint32_t wpId);
  

  struct e01_load_request_arrived {}; 
  struct e02_load_done {};
  struct e02_upload_done {};
  struct e02_load_failed {};
  struct e03_start_arrived
  {
  };
  struct e04_pre_flight_check_completed
  {
  };
  struct e04_pre_flight_check_failed
  {
  };
  struct e06_takeoff_received
  {
  };
  struct e07_towards_first_wp {};
  struct e08_waypoint_reached
  {
  };
  struct e09_last_waypoint_reached
  {
  };
  struct e10_landed
  {
  };

  struct e0x_fcs_present
  {};

  struct e0x_fcs_not_present
  {};

  struct PlanExecutorStateMachine
  {
    auto operator()() noexcept
    {
      using namespace boost::sml;

      return make_transition_table(
          *"init"_s + event<e01_load_request_arrived> /
                    [](PlanExecutorModule &planexec) { planexec.notifyLoadingPlan(); } = "loading_plan"_s,
          "loading_plan"_s + event<e02_load_done> /
                    [](PlanExecutorModule &planexec) { planexec.checkFCS(); } = "checking_fcs"_s,
          "loading_plan"_s + event<e02_load_failed> /
                    [](PlanExecutorModule &planexec) { planexec.notifyLoadPlanFailed(); } = "done"_s,
          "checking_fcs"_s + event<e0x_fcs_present> = "uploading_plan_to_fcs"_s,
          "checking_fcs"_s + event<e0x_fcs_not_present> /
                    [](PlanExecutorModule &planexec) { planexec.notify(StdMessage::PLAN_EXEC_FCS_NOT_RESPONDING_NOTIFICATION); } = "done"_s,
          "uploading_plan_to_fcs"_s + event<e02_upload_done> /
                    [](PlanExecutorModule &planexec) { planexec.notifyPlanLoaded(); } = "plan_loaded"_s,
          "plan_loaded"_s + event<e03_start_arrived> /
                    [](PlanExecutorModule &planexec) { planexec.notify(StdMessage::PLAN_EXEC_PREFLIGHT_TESTING_NOTIFICATION); } = "preflight_tests"_s,
          "preflight_tests"_s + event<e04_pre_flight_check_completed> /
                    [](PlanExecutorModule &planexec) { 
                      planexec.notify(StdMessage::PLAN_EXEC_READY_TO_FLIGHT_NOTIFICATION);
                      planexec.checkFirstWPCloseness();
                       } = "wait_to_takeoff"_s,
          "preflight_tests"_s + event<e04_pre_flight_check_failed> /
                    [](PlanExecutorModule &planexec) { planexec.notifyPreflightChecksFailed(); } = "done"_s,
          "wait_to_takeoff"_s + event<e06_takeoff_received> /
                    [](PlanExecutorModule &planexec) { planexec.notify(StdMessage::PLAN_EXEC_LAUNCHED_NOTIFICATION); } = "taking_off"_s,
          "taking_off"_s + event<e07_towards_first_wp> = "flying_to_next_waypoint"_s,
          "flying_to_next_waypoint"_s + event<e08_waypoint_reached> /
                    [](PlanExecutorModule &planexec) { planexec.notifyWaypointReached(); } = "flying_to_next_waypoint"_s,
          "flying_to_next_waypoint"_s + event<e09_last_waypoint_reached> /
                    [](PlanExecutorModule &planexec) { planexec.notify(StdMessage::PLAN_EXEC_LANDING_NOTIFICATION); } = "landing"_s,
          "landing"_s + event<e10_landed> /
                    [](PlanExecutorModule &planexec) { planexec.notify(StdMessage::PLAN_EXEC_FLIGHT_PLAN_COMPLETED_NOTIFICATION); } = "done"_s,
          "done"_s /
                    [](PlanExecutorModule &planexec) { planexec.finishExecution(); } = X);
  }
};

#ifdef _DEBUG
struct StateMachineLogger
{
  template <class SM, class TEvent>
  void log_process_event(const TEvent &)
  {
    Log::info("[process_event] {}", typeid(TEvent).name());
  }

  template <class SM, class TGuard, class TEvent>
  void log_guard(const TGuard &, const TEvent &, bool result)
  {
    Log::info("[guard] {} {} {}", typeid(TGuard).name(), typeid(TEvent).name(),
              (result ? "[OK]" : "[Reject]"));
  }

  template <class SM, class TAction, class TEvent>
  void log_action(const TAction &, const TEvent &)
  {
    Log::info("[action] {} {}", typeid(TAction).name(), typeid(TEvent).name());
  }

  template <class SM, class TSrcState, class TDstState>
  void log_state_change(const TSrcState &src, const TDstState &dst)
  {
    Log::info("[transition] {} -> {}", src.c_str(), dst.c_str());
  }
};
StateMachineLogger smlogger;
std::unique_ptr<boost::sml::sm<PlanExecutorStateMachine, boost::sml::logger<StateMachineLogger>>> statemachine;
#else
std::unique_ptr<boost::sml::sm<PlanExecutorStateMachine>> statemachine;
#endif
};
}
;
