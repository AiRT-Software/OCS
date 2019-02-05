
#pragma pack(push, 1)

/*
    This module is in charge of executing a flight plan, that is, turning the curve defined by the user in the GCS
    into a sequence of waypoints for the FCS and a sequence of commands for the gimbal.

    The steps for using this module are:

    1. Change the status of atreyu to RECORDING_STATE. (Sending anything to this module in other state will result in an INVALID_STATUS error)
    2. Ask the plan executor module to load a flight plan from a mission (PLAN_EXEC_LOAD_PLAN)(guid, plan_index)
    3. If the current position of the drone is "far" from the first waypoint, a notification will be issued
    (PLAN_EXEC_FIRST_WAYPOINT_IS_TOO_FAR_NOTIFICATION), otherwise a PLAN_EXEC_READY_TO_FLIGHT_NOTIFICATION will be issued
    4. When the user is ready, she will start the plan (PLAN_EXEC_START)
    5. After we start flying, the fcs module and the pozyx module will start publishing updated info
    6. When reaching a waypoint, a PLAN_EXEC_REACHED_WAYPOINT_NOTIFICATION will be published
    7. The user can control the flight issuing commands to pausing it (PLAN_EXEC_PAUSE), resuming a paused flight (PLAN_EXEC_RESUME), or
    canceling it (PLAN_EXEC_STOP) (it changes flight mode to loiter).
    8. If the drone arrives to the last waypoint, it will land automatically and it will issue a (PLAN_EXEC_FLIGHT_PLAN_COMPLETED_NOTIFICATION)



*/


    // ACTIONS
    // Non payload actions are built using a single AiRT command packet:
    // { 'A', OS_MODULE, <ACTION>}
    // where <ACTION> can be any of the \sa StdMessage::PlanExecutorCommandType that do not require a parameter

    // Send the plan base name to upload to the fcs. Two-part message. The second part has the mission base name
    struct LoadPlan {
        const AIRT_Message_Header header{StdMessage::PLAN_EXECUTOR_MODULE, StdMessage::PLAN_EXEC_LOAD_PLAN};
        unsigned char plan; // Which plan in the mission? 0..<plans-1>
    };

    // Send the registration matrix
    struct RegistrationMatrix {
        const AIRT_Message_Header header{StdMessage::PLAN_EXECUTOR_MODULE, StdMessage::PLAN_EXEC_SET_REGISTRATION_MATRIX};
        unsigned char rowMajor; // 1 if the matrix is given by rows, 0 if it is given by columns
        float elems[16];
    };



    // NOTIFICATIONS
    struct RegistrationMatrixChanged {
        const AIRT_Message_Header header{StdMessage::PLAN_EXECUTOR_NOTIFICATIONS_MODULE, StdMessage::PLAN_EXEC_REGISTRATION_MATRIX_CHANGED_NOTIFICATION};
        unsigned char  rowMajor; // 1 if the matrix is given by rows, 0 if it is given by columns
        float elems[16];
    };

    struct ReachedWaypoint {
        const AIRT_Message_Header header{StdMessage::PLAN_EXECUTOR_NOTIFICATIONS_MODULE, StdMessage::PLAN_EXEC_REACHED_WAYPOINT_NOTIFICATION};
        unsigned char space; // not used
        uint16_t wp;
    };

    // Two part message. The second part has the GUID (base name) of the mission in asciiz
    struct CurrentFlightPlan {
        const AIRT_Message_Header header{StdMessage::PLAN_EXECUTOR_NOTIFICATIONS_MODULE, StdMessage::PLAN_EXEC_CURRENT_FLIGHT_PLAN_NOTIFICATION};
        unsigned char space; // not used
        uint16_t planIndex; // Index of the plan inside of the mission (0-based)
    };
#pragma pack(pop)