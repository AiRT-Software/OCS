
#pragma pack(push, 1)
    // ACTIONS
    // Non payload actions are built using a single AiRT command packet:
    // { 'A', OS_MODULE, <ACTION>}
    // where <ACTION> can be any of the \sa StdMessage::AtreyuCommandType that do not require a parameter

    /*
    
     ***APPLICATION STATES***

     There are only three system states:

        IDLE_STATE
        MAPPING_STATE
        RECORDING_STATE

    IDLE_STATE ocurrs when starting atreyu and when we are done recording or mapping. The clients can request to change 
    to MAPPING or RECORDING when the system is IDLE. In any other case, an error will be issued and the system state will not change.
    When MAPPING or RECORDING end, the state changes automatically back to IDLE.
    Whenever a change ocurrs, an ATREYU_VERSION_NOTIFICATION notification is issued (that specifies the old and the new states). Clients
    can also request the current state at any time. In that case, both old and new states are equal.
    */

   


    // NOTIFICATIONS

    // Server version
    struct AtreyuVersionNotification
    {
        const AIRT_Message_Header header{StdMessage::ATREYU_NOTIFICATIONS_MODULE, StdMessage::ATREYU_VERSION_NOTIFICATION};
        uint8_t major;
        uint8_t minor;
        uint8_t patch;
    };

    // Two-part message.
    struct AtreyuUpdatePackagePathnameNotification
    {
        const AIRT_Message_Header header{StdMessage::ATREYU_NOTIFICATIONS_MODULE, StdMessage::ATREYU_UPDATE_PACKAGE_PATHNAME_NOTIFICATION};
    };
    // Second part: ascii-z with the full pathname (directory+filename) where to copy the server package with the OS actions    


    // Notification of the current Atreyu's state. The message contains both the new and old state. If someone asked for the current
    // state of Atreyu (for example, when a new client joins an existing session, the oldstate and the newstate will be the same)
    struct AtreyuStateChanged
    {
        const AIRT_Message_Header header{StdMessage::ATREYU_NOTIFICATIONS_MODULE, StdMessage::ATREYU_SYSTEM_STATE_CHANGE_NOTIFICATION};
        uint8_t oldstate, newstate;
    };
#pragma pack(pop)