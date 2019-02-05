#include "globalSettings.h"
#include "planexecutormodule.h"

#include <sstream>
#include <log.h>
#include <stdMessage.h>
#include <utils.h>

using airt::AIRT_Message_Header;
using airt::StdMessage;

#include "interface/atreyumodule.h"
#include "interface/planlibrarianmodule.h"
#include "interface/pozyxmodule.h"
#include "interface/fcsmodule.h"
#include "interface/gimbalmodule.h"

using airt::Log;
using airt::PlanExecutorModule;
using airt::StdMessage;

using airt::MissionLoader;

PlanExecutorModule::PlanExecutorModule(const std::string &portname, const std::string &pubname, const std::string &subname,
                                       std::shared_ptr<Context> context)
    : Module(portname, pubname, subname, context)
{
    assert(context);

    if (!GlobalSettings::getValue("plan_exec_preflight_check_timeout_seconds", preflightCheckTimeout))
    {
        preflightCheckTimeout = 3.0f;
        Log::warn("Time out for preflight checks not defined (plan_exec_preflight_check_timeout_seconds). Set to {} s", preflightCheckTimeout);
    }

    if (!GlobalSettings::getValue("plan_exec_motor_power_to_arm", motorsArmPowerThreshold))
    {
        motorsArmPowerThreshold = 30.0f;
        Log::warn("Treshold for considering the drone armed no defined (plan_exec_motor_power_to_arm)). Set to {} %", motorsArmPowerThreshold);
    }

    if (!GlobalSettings::getValue("plan_exec_bypass_preflight_checks", bypassPreflightChecks))
    {
        preflightCheckTimeout = false;
        Log::warn("Bypass preflight checks not defined (plan_exec_bypass_preflight_checks)). Set to {}", bypassPreflightChecks);
    }

    if (!GlobalSettings::getValue("plan_exec_maximum_distance_to_first_wp_meters", warningDistanceToFirstWPMeters))
    {
        warningDistanceToFirstWPMeters = 2.0f;
        Log::warn("Maximum distance to first waypoint not defined (plan_exec_maximum_distance_to_first_wp_meters)). Set to {}", warningDistanceToFirstWPMeters);
    }

    if (!GlobalSettings::getValue("plan_exec_minimum_takeoff_height_meters", minimumTakeOffHeightMeters))
    {
        minimumTakeOffHeightMeters = 1.5f;
        Log::warn("Minimum takeoff height not defined (plan_exec_minimum_takeoff_height_meters)). Set to {}", minimumTakeOffHeightMeters);
    }

    fcsPresent = bypassPreflightChecks;
    Log::info("Plan executor module initialized");
}

void PlanExecutorModule::configureSubscriptions(const std::string &mainpublisher)
{
    assert(subsocket);
    subsocket->connect(mainpublisher);
    uint8_t fcssignature[]{'A', StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE};
    subsocket->subscribe(fcssignature, airt::arraySize(fcssignature));
    uint8_t pozyxsignature[]{'A', StdMessage::POSITIONING_NOTIFICATIONS_MODULE};
    subsocket->subscribe(pozyxsignature, airt::arraySize(pozyxsignature));
    uint8_t gimbalsignature[]{'A', StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE};
    subsocket->subscribe(gimbalsignature, airt::arraySize(gimbalsignature));
    uint8_t atreyusignature[]{'A', StdMessage::ATREYU_NOTIFICATIONS_MODULE};
    subsocket->subscribe(atreyusignature, airt::arraySize(atreyusignature));
    uint8_t planlibsignature[]{'A', StdMessage::PLAN_LIBRARIAN_NOTIFICATIONS_MODULE};
    subsocket->subscribe(planlibsignature, airt::arraySize(planlibsignature));
}

void PlanExecutorModule::reset()
{
    statemachine.reset();
    missionBaseName.clear();
    missionFilename.clear();
    planIndex = 0;
    preflightCheckErrorString.clear();
    mapToPozyx = boost::none;
    fcsPresent = bypassPreflightChecks;
    gimbalFlightPlan.clear();
    lastWPReached = 0;
}

void PlanExecutorModule::checkFCS()
{
    using namespace boost::sml;

    // Check the FCS
    MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);
    airt::StdMessage ping(StdMessage::FCS_MULTIPLEXER_MODULE, StdMessage::FCS_PING);
    airt::sendTwoPartMessage(*cmdsocket, &header, &ping);

    schedule(1000, [this]() {
        if (!fcsPresent && statemachine->is("checking_fcs"_s))
            statemachine->process_event(e0x_fcs_not_present{});
    });
}

void PlanExecutorModule::onNotification(const Message &m)
{
    using namespace boost::sml;

    uint8_t module = airt::getMessageModule(m);
    uint8_t action = airt::getMessageAction(m);

    if (!statemachine)
    {
        // the module is not active
        if (module == StdMessage::ATREYU_NOTIFICATIONS_MODULE && action == StdMessage::ATREYU_SYSTEM_STATE_CHANGE_NOTIFICATION)
        {
            // are we going to record?
            const auto *sc = m.get<const AtreyuStateChanged *>(0);
            if (sc->newstate != StdMessage::RECORDING_STATE)
                return;
        }
        else
            return;
    }

    Log::info("PLAN EXECUTOR: Notification from {}. Action: {}", module, action);

    switch (module)
    {
    case StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE:
        switch (action)
        {
        case StdMessage::FCS_PONG_NOTIFICATION:
        {
            fcsPresent = true;

            Log::info("The FCS is responding.");
            if (statemachine->is("checking_fcs"_s))
            {
                statemachine->process_event(e0x_fcs_present{});
            }
        }
        break;
        case StdMessage::FCS_MOTORS_NOTIFICATION:
        {
            const MotorSpeedNotification *ms = m.get<const MotorSpeedNotification *>(0);
            motorsThrottle = ms->power;
            if (motorsThrottle < 2 && statemachine->is("landing"_s))
            {
                statemachine->process_event(e10_landed{});
            }
        }
        break;
        case StdMessage::FCS_WPREACHED_NOTIFICATION:
        {
            const WaypointReachedNotification *wr = m.get<const WaypointReachedNotification *>(0);
            lastWPReached = wr->waypoint;

            processWaypointPayload(wr->waypoint);

            if (wr->waypoint == map.paths[planIndex].waypoints.size())
            { // which wp is the last? N or N-1?? -> N
                statemachine->process_event(e09_last_waypoint_reached{});
                land();
            }
            else
            {
                statemachine->process_event(e08_waypoint_reached{});
            }
        }
        break;
        case StdMessage::FCS_WPREQUEST_NOTIFICATION:
        {
            const RequestWaypointNotification *wpr = m.get<const RequestWaypointNotification *>(0);
            auto waypointId = wpr->waypoint;
            if (waypointId > map.paths[planIndex].waypoints.size())
            {
                Log::critical("The requested waypoint ({}) does not exists. Ignoring request", waypointId);
                Log::critical("This is a know bug of Aerotools' multiplexer. Generating an e02_upload_done event (generated upon receiving MissionACK");
                statemachine->process_event(e02_upload_done{});
                return;
            }
            MissionContainer::WayPoint wp = map.paths[planIndex].waypoints[waypointId - 1];
            glm::vec4 xformed = (*mapToPozyx) * glm::vec4(wp.x, wp.y, wp.z, 1.0);

            UploadWaypoint fcswp;
            fcswp.id = waypointId;
            fcswp.x = xformed.x;
            // Watch out!!!! TODO
            fcswp.y = xformed.z;
            fcswp.z = xformed.y;

            Log::info("Waypoint #{} sent to the fcs. Map coords ({}, {}, {}), Pozyx coords ({}, {}, {})", waypointId,
                      wp.x, wp.y, wp.z, xformed.x, xformed.y, xformed.z);
            MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);
            airt::sendTwoPartMessage(*cmdsocket, &header, &fcswp);
        }
        break;
        case StdMessage::FCS_MISSIONACK_NOTIFICATION:
        {
            statemachine->process_event(e02_upload_done{});
        }
        break;
        }
        break;
    case StdMessage::POSITIONING_NOTIFICATIONS_MODULE:
    {
        if (action == StdMessage::IPS_DATA)
        {
            const PositioningFrame *p = m.get<const PositioningFrame *>(0);
            currentPositionIPS = glm::vec4(p->x, p->y, p->z, 1.0f);
            rollIPS = p->roll;
            yawIPS = p->yaw;
            pitchIPS = p->pitch;
            timestampIPS = p->timestamp;
            timestampIPSMyTime = std::chrono::steady_clock::now();
        }
    }
    break;
    case StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE:
        break;
    case StdMessage::ATREYU_NOTIFICATIONS_MODULE:
        if (action == StdMessage::ATREYU_SYSTEM_STATE_CHANGE_NOTIFICATION)
        {
            const auto *sc = m.get<const AtreyuStateChanged *>(0);
            if (sc->newstate == StdMessage::IDLE_STATE && statemachine)
            {
                Log::info("Plan executor is done");
                reset();
            }
            else if (sc->newstate == StdMessage::RECORDING_STATE && !statemachine)
            {
                Log::info("Plan executor starts");
#ifdef _DEBUG
                statemachine = std::unique_ptr<boost::sml::sm<PlanExecutorStateMachine, logger<StateMachineLogger>>>(new boost::sml::sm<PlanExecutorStateMachine, logger<StateMachineLogger>>{*this, smlogger});
#else
                statemachine = std::unique_ptr<boost::sml::sm<PlanExecutorStateMachine>>(new boost::sml::sm<PlanExecutorStateMachine>(*this));
#endif
            }
        }
        break;
    case StdMessage::PLAN_LIBRARIAN_NOTIFICATIONS_MODULE:
    {
        if (action == StdMessage::PLAN_LIB_MISSION_NOTIFICATION)
        {
            missionFilename = m.get<const char *>(2);
            if (loadPlan())
            {
                Log::info("Mission {} loaded", missionBaseName);
                if (planIndex >= map.paths.size())
                {
                    Log::error("The plan index requested ({}) does not exists. There are {} plans", planIndex, map.paths.size());
                    statemachine->process_event(e02_load_failed{});
                }
                else
                {
                    statemachine->process_event(e02_load_done{});
                    uploadPlanToFCS();
                }
            }
            else
            {
                Log::error("Mission {} failed to load", missionBaseName);
                statemachine->process_event(e02_load_failed{});
            }
        }
        else if (action == StdMessage::PLAN_LIB_FILE_NOT_FOUND_NOTIFICATION && statemachine->is("loading_plan"_s))
        {
            Log::error("File not found: {}", missionFilename);
            statemachine->process_event(e02_load_failed{});
        }
    }
    break;

    default:
        Log::critical("Error: message from unexpected module");
    }
}

void PlanExecutorModule::processWaypointPayload(uint32_t wpId)
{

    // Depending on waypoint type...
    auto waypoint = map.getWaypoint(planIndex, wpId);
    switch (waypoint.type)
    {
    case MissionContainer::TAKEOFF:
        Log::error("Ignoring take off waypoint");
        break;
    case MissionContainer::WAYPOINT:
        break;
    case MissionContainer::STOP:
        Log::error("Ignoring stop waypoint");
        break;
    case MissionContainer::LAND:
        Log::error("Ignoring land waypoint");
        break;
    }
    // Send waypoint speed, if it has changed
    if (wpId == 0 || waypoint.speed != map.getWaypoint(planIndex, wpId - 1).speed)
    {
        MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);
        SetFCSSpeed fcss;
        fcss.speed = static_cast<uint8_t>(waypoint.speed + 0.5f);
        airt::sendTwoPartMessage(*cmdsocket, &header, &fcss);
    }

    // Send camera actions
    processRCamCommands(wpId);

    // Send gimbal actions
    processGimbalCommands(wpId);
}

void PlanExecutorModule::processRCamCommands(uint32_t wpId)
{
    int index = map.findRecCamParameters(planIndex, wpId);
    if (index < 0)
    {
        // Nothing to do
        return;
    }
    // Execute the commands
    auto &commands = map.getRecCamParameters(planIndex, index).commands;
    std::vector<uint8_t> tmp;
    tmp.resize(64);

    Log::info("Waypoint #{} has {} commands for the RCam", wpId, commands.size());
    for (auto cmd : commands)
    {
        // Send this command to the camera
        MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);
        tmp.push_back('A');
        tmp.push_back(StdMessage::RCAM_MODULE);
        std::copy(cmd.begin(), cmd.end(), tmp.begin() + 2);
        airt::sendTwoPartMessage(*cmdsocket, &header, &tmp[0], tmp.size());
        tmp.clear();
    }
}

void PlanExecutorModule::processGimbalCommands(uint32_t wpId)
{

    Log::info("Processing gimbal command for WP {}", wpId);
    if (wpId < gimbalFlightPlan.size())
    {
        MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);
        GoToAngle gta;
        gta.pitch = gimbalFlightPlan[wpId].pitch;
        gta.roll = gimbalFlightPlan[wpId].roll;
        gta.yaw = gimbalFlightPlan[wpId].yaw;
        gta.speedRadPerSec = gimbalFlightPlan[wpId].speed;

        airt::sendTwoPartMessage(*cmdsocket, &header, &gta, sizeof(gta));
    }
}

void PlanExecutorModule::preflightChecks()
{
    // to flight a mission we absolutely need: the registration matrix (map -> pozyx), ips, fcs, gimbal

    if (bypassPreflightChecks)
    {
        Log::warn("Bypassing preflight checks");
        statemachine->process_event(e04_pre_flight_check_completed{});
        return;
    }

    Log::info("Preflight checks...");
    // The loaded mission has the plan and the plan has waypoints
    if (planIndex >= map.paths.size())
    {
        preflightCheckErrorString = "The selected plan does not exist in the mission";
        statemachine->process_event(e04_pre_flight_check_failed{});
        return;
    }
    if (map.paths[planIndex].waypoints.size() < 2)
    {
        preflightCheckErrorString = "The selected plan has less than two waypoints";
        statemachine->process_event(e04_pre_flight_check_failed{});
        return;
    }

    // Is the registration matrix ready?
    if (!mapToPozyx)
    {
        preflightCheckErrorString = "The registration matrix has not been set.";
        statemachine->process_event(e04_pre_flight_check_failed{});
        return;
    }
    Log::info("Registration matrix has been registered");

    // Check the IPS
    std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - timestampIPSMyTime;
    if (elapsed.count() >= preflightCheckTimeout)
    {
        // If its been more than three seconds since we last saw a positioning frame, it is not working
        preflightCheckErrorString = "The IPS does not respond";
        statemachine->process_event(e04_pre_flight_check_failed{});
        return;
    }
    Log::info("The IPS system is working");

    // Check the gimbal
    // Currently there is no way to check the gimbal

    // FCS present?
    // already checked

    statemachine->process_event(e04_pre_flight_check_completed{});
}

void PlanExecutorModule::arm()
{
    using namespace boost::sml;
    if (!statemachine->is("taking_off"_s))
    {
        Log::error("Trying to arm in the wrong state. Ignoring");
        return;
    }
    MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);
    airt::StdMessage arm(StdMessage::FCS_MULTIPLEXER_MODULE, StdMessage::FCS_ARM);
    airt::sendTwoPartMessage(*cmdsocket, &header, &arm);
}

void PlanExecutorModule::takeoff()
{
    using namespace boost::sml;
    if (!statemachine->is("taking_off"_s))
    {
        Log::error("Trying to take off in the wrong state. Ignoring");
        return;
    }
    MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);

    auto firstWPMap = map.paths[planIndex].waypoints[0];
    glm::vec4 firstWPPozyx = (*mapToPozyx) * glm::vec4(firstWPMap.x, firstWPMap.y, firstWPMap.z, 1.0f);

    // Take off to the greatest of: height of the first WP or minimumTakeOffHeight
    TakeOff to;
    to.height = std::max(firstWPPozyx.z, minimumTakeOffHeightMeters /** 1000.f*/);
    airt::sendTwoPartMessage(*cmdsocket, &header, &to);
}

void PlanExecutorModule::fly()
{
    using namespace boost::sml;
    if (!statemachine->is("taking_off"_s))
    {
        Log::error("Trying to fly in the wrong state. Ignoring");
        return;
    }

    gimbalFlightPlan = map.compileGimbalFlightPlan(planIndex);

    MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);
    SetModeFCS fmode;
    fmode.mode = StdMessage::MODE_AUTO;
    airt::sendTwoPartMessage(*cmdsocket, &header, &fmode);
}

void PlanExecutorModule::land()
{
    MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);
    StdMessage land(StdMessage::FCS_MULTIPLEXER_MODULE, StdMessage::FCS_LAND);
    airt::sendTwoPartMessage(*cmdsocket, &header, &land);
    // Do I need to disarm? -> NO
}

bool PlanExecutorModule::loadPlan()
{
    if (!MissionLoader::loadMap(missionFilename, map))
    {
        return false;
    }
    Log::info("PlanExecutorModule: map was loaded: num_paths {}", map.paths.size());
    return true;
}

void PlanExecutorModule::uploadPlanToFCS()
{
    MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);
    StdMessage clearall(StdMessage::FCS_MULTIPLEXER_MODULE, StdMessage::FCS_CLEARALL);
    airt::sendTwoPartMessage(*cmdsocket, &header, &clearall);

    CreateMission cm;
    cm.numberOfWP = map.paths[planIndex].waypoints.size();
    airt::sendTwoPartMessage(*cmdsocket, &header, &cm);
}

void PlanExecutorModule::notify(airt::StdMessage::PlanExecutorNotificationType notification)
{
    airt::sendNotification(*pubsocket, StdMessage::PLAN_EXECUTOR_NOTIFICATIONS_MODULE, notification);
}

void PlanExecutorModule::notify(airt::StdMessage::PlanExecutorNotificationType notification, const std::string &data)
{
    StdMessage header(StdMessage::PLAN_EXECUTOR_NOTIFICATIONS_MODULE, notification);
    airt::sendTwoPartMessage(*pubsocket, &header, data);
}
void PlanExecutorModule::notifyLoadingPlan()
{
    notify(StdMessage::PLAN_EXEC_PLAN_LOADING_NOTIFICATION, missionBaseName);
}

void PlanExecutorModule::notifyLoadPlanFailed()
{
    notify(StdMessage::PLAN_EXEC_ERROR_LOADING_PLAN_NOTIFICATION, missionBaseName);
}

void PlanExecutorModule::notifyPlanLoaded()
{
    notify(StdMessage::PLAN_EXEC_PLAN_LOADED_NOTIFICATION, missionBaseName);
}

void PlanExecutorModule::notifyPreflightChecksFailed()
{
    notify(StdMessage::PLAN_EXEC_PREFLIGHT_TESTS_FAILED_NOTIFICATION, preflightCheckErrorString);
}

void PlanExecutorModule::notifyWaypointReached()
{
    ReachedWaypoint rwp;
    rwp.wp = lastWPReached;
    airt::sendOnePartMessage(*pubsocket, &rwp);
}

void PlanExecutorModule::onIdle()
{
    if (!statemachine)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return;
    }
}

void PlanExecutorModule::onMessage(const Message &inmsg)
{
    using namespace boost::sml;

    if (!statemachine)
    {
        Log::error("Message received when not in RECORDING mode {}", airt::to_string(inmsg));
        airt::sendNotification(*pubsocket, StdMessage::PLAN_EXECUTOR_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
        return;
    }
    auto action = airt::getMessageAction(inmsg);
    switch (action)
    {
    case StdMessage::PLAN_EXEC_SET_REGISTRATION_MATRIX:
    {
        Log::info("Received registration matrix");
        const auto m = inmsg.get<const RegistrationMatrix *>(0);
        glm::mat4 mat(m->elems[0], m->elems[1], m->elems[2], m->elems[3], m->elems[4], m->elems[5], m->elems[6], m->elems[7],
                      m->elems[8], m->elems[9], m->elems[10], m->elems[11], m->elems[12], m->elems[13], m->elems[14], m->elems[15]);

        std::ostringstream matElems;
        matElems << "matrix: {\n";
        for (size_t i = 0; i < 16; i++)
        {
            matElems << m->elems[i] << " ";
            if (i % 4 == 3)
                matElems << "\n";
        }
        matElems << "}" << (m->rowMajor ? "ROW-MAJOR" : "COLUMN-MAJOR");
        Log::info(matElems.str());

        if (m->rowMajor)
            mapToPozyx = glm::transpose(mat);
        else
            mapToPozyx = mat;
        RegistrationMatrixChanged mp;
        mp.rowMajor = m->rowMajor;
        memcpy(mp.elems, m->elems, sizeof(float) * 16);
        airt::sendOnePartMessage(*pubsocket, &mp);
        break;
    }
    case StdMessage::PLAN_EXEC_LOAD_PLAN:
    {
        const auto m = inmsg.get<const LoadPlan *>(0);
        missionBaseName = inmsg.get<const char *>(1);
        planIndex = m->plan;
        Log::info("Request for loading plan {} from mission {}", planIndex, missionBaseName);
        MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);
        RequestMission reqMission;
        airt::sendThreePartMessage(*cmdsocket, &header, &reqMission, missionBaseName);
        statemachine->process_event(e01_load_request_arrived{});
        break;
    }
    case StdMessage::PLAN_EXEC_REQUEST_CURRENT_FLIGHT_PLAN:
    {
        Log::info("Request of the current flight plan");
        CurrentFlightPlan cfp;
        cfp.planIndex = planIndex;
        sendTwoPartMessage(*pubsocket, &cfp, missionBaseName);
        break;
    }
    case StdMessage::PLAN_EXEC_START:
    {
        Log::info("Received the start mission command.");
        statemachine->process_event(e03_start_arrived{});
        preflightChecks();
        break;
    }

    case StdMessage::PLAN_EXEC_TAKEOFF:
    {
        Log::info("Received the take off command. Arming...");
        statemachine->process_event(e06_takeoff_received{});
        arm();
        processGimbalCommands(0);
        processRCamCommands(0);
        schedule(3000, [this]() {
            Log::info("Sending the takeoff command and waiting 3 s for starting the flight plan");
            takeoff();
            schedule(3000, [this]() {
                Log::info("Starting the flight plan");
                fly();
                statemachine->process_event(e07_towards_first_wp{});
            });
        });
        break;
    }
    case StdMessage::PLAN_EXEC_STOP:
    {
        Log::error("There is still not use for the PLAN_EXEC_STOP command");
        // if (statemachine->is("flying_to_next_waypoint"_s)) {
        //     land();
        // }
        // reset();
        // TODO: what happens when we are flying?
        // Cleanup the mess...
    }
    break;
    case StdMessage::PLAN_EXEC_EXIT:
    {
        finishExecution();
        break;
    }
    default:
        Log::critical("Unhandled message in plan librarian module");
    }
}

void PlanExecutorModule::finishExecution()
{
    using namespace boost::sml;

    Log::info("Plan exec: finishing execution");

    // Cleanup the mess...
    if (statemachine->is("flying_to_next_waypoint"_s))
    {
        land();
    }
    MsgToOtherModule header(StdMessage::PLAN_EXECUTOR_MODULE);
    StdMessage gotoIdle(StdMessage::ATREYU_MODULE, StdMessage::ATREYU_LAST_COMMAND + 1);
    airt::sendTwoPartMessage(*cmdsocket, &header, &gotoIdle, sizeof(gotoIdle));
    reset();
}

void PlanExecutorModule::checkFirstWPCloseness()
{
    using namespace boost::sml;

    if (statemachine->is("wait_to_takeoff"_s))
    {
        auto firstWPMap = map.paths[planIndex].waypoints[0];
        glm::vec4 firstWPPozyx = (*mapToPozyx) * glm::vec4(firstWPMap.x, firstWPMap.y, firstWPMap.z, 1.0f);
        if (glm::distance(glm::vec3(firstWPPozyx), glm::vec3(currentPositionIPS)) > warningDistanceToFirstWPMeters)
            notify(StdMessage::PLAN_EXEC_FIRST_WAYPOINT_IS_TOO_FAR_NOTIFICATION);
        // repeat the warning every 2 s (if we move the drone close to the first wapoint, the warning should stop)
        schedule(2000, [this]() {
            checkFirstWPCloseness();
        });
    }
}
