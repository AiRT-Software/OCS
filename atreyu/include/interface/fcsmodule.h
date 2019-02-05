#pragma pack(push, 1)
    // ACTIONS
    // Non payload actions are built using a single AiRT command packet:
    // { 'A', FCS_MODULE, <ACTION>}
    // where <ACTION> can be any of the \sa StdMessage::FCSMultiplexerCommandType that do not require a parameter

    // Change the flight mode 
    struct SetFCSMode {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_MODULE, StdMessage::FCS_SETMODE};
        uint8_t mode;   // See FCSFlightModes. Watch out! Basically we will use MODE_AUTO and LOITER?    
    };

    // Go to a certain location (in pozyx space)
    struct GotoLocation {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_MODULE, StdMessage::FCS_GOTO};
        uint8_t space;   // not used
        float x, y, z; // in pozyx space, mm
    };

    // Take off command
    struct TakeOff {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_MODULE, StdMessage::FCS_TAKEOFF};
        uint8_t space; // not used
        float height; // in poxyz space mm
    };

    struct UploadWaypoint {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_MODULE, StdMessage::FCS_WP};
        uint8_t space; // not used
        uint16_t id;
        float x, y, z;  // in pozyx space, mm
    };

    struct SetFCSSpeed {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_MODULE, StdMessage::FCS_SETSPEED};
        uint8_t speed;  // in m/s
    };

    // For other actions:
    // Set the flight mode in the FCS
    struct SetModeFCS {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_MODULE, StdMessage::FCS_SETMODE};
        uint8_t mode;
    };

    // Creates a mission in the FCS (it will later request the waypoints)
    struct CreateMission {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_MODULE, StdMessage::FCS_CREATEMISSION};
        uint8_t space; // not used
        uint16_t numberOfWP;
    };




    // NOTIFICATIONS
    // The FCS Multiplexer acknowledges reception of the given command
    struct AckNotification
    {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_ACK_NOTIFICATION};
        uint8_t command;
    };

    // The FCS Multiplexer rejected of the given command
    struct NackNotification
    {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_NACK_NOTIFICATION};
        uint8_t command;
    };

    // The FCS Multiplexer informs the drone has arrived to the given waypoint
    struct WaypointReachedNotification
    {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_WPREACHED_NOTIFICATION};
        uint16_t waypoint;
    };

    // The FCS Multiplexer request the information for the given waypoint
    struct RequestWaypointNotification
    {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_WPREQUEST_NOTIFICATION};
        uint16_t waypoint;
    };


    // The FCS Multiplexer provides the current roll, pitch and yaw
    struct RollPitchYawNotification
    {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_ROLLPITCHYAW_NOTIFICATION};
        float roll, pitch, yaw; // in radians
    };

    // The FCS Multiplexer provides the current motor power
    struct MotorSpeedNotification
    {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_MOTORS_NOTIFICATION};
        uint8_t power; // in percent (0..100)
    };

    // The FCS Multiplexer provides the current battery level
    struct BatteryLevelNotification
    {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_BATTERY_NOTIFICATION};
        uint8_t level; // in percent (0..100)
    };

    // The FCS Multiplexer provides the current position
    struct FCSPositionNotification
    {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_DATA_POS_NOTIFICATION};
        float x, y, z; // position in pozyx space, in mm
    };

    // A new waypoint has been uploaded to the FCS
    struct FCSUploadedWaypoint {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_UPLOADED_WAYPOINT_NOTIFICATION};
        uint8_t space; // not used
        uint16_t id;
        float x, y, z;  // in pozyx space, mm
    };

    // The drone speed has been requested to change to a new value
    struct SpeedChanged {
        const AIRT_Message_Header header{StdMessage::FCS_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::FCS_SPEED_CHANGED_NOTIFICATION};
        uint8_t space; // not used
        float speed;  // in m/s
    };

#pragma pack(pop)
