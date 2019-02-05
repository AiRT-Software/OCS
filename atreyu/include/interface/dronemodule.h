#pragma pack(push, 1)

/*

The drone position and orientation are defined in a right-handed coordinate system.
The position is defined in the Poyx coordinate system:

X, Y: horizontal plane. Y defines the NORTH direction, X defines the EAST direction
Z: points up

All units in mm.

The drone defines its local coordinate system with those same directions:

Y: points to the front of the drone (where the depth camera is located)
Z: points up
X: points right

pitch: right-handed rotation about the local X axis. Positive nose up
roll: right-handed rotation about the local Y axis. Tilt right is positive
yaw: angle between the world Y axis (north direction) and the Y local axis. Increases looking to the left

All angles in radians.

//// TODO: gimbal, reccam

*/



// ACTIONS
// Non payload actions are built using a single AiRT command packet:
// { 'A', DRONE_MODULE, <ACTION>}
// where <ACTION> can be any of the \sa StdMessage::DroneCommandType that do not require a parameter

struct SetZCameraOffset {
    const AIRT_Message_Header header{StdMessage::DRONE_MODULE, StdMessage::DRONE_SET_ZCAMERA_OFFSET};
    uint8_t space; // not used
    float offsetX, offsetY, offsetZ;  // Position of the z-camera in the drone's local coordinate system
};



// NOTIFICATIONS

struct DroneZCamOffsetNotification {
    const AIRT_Message_Header header{StdMessage::DRONE_NOTIFICATIONS_MODULE, StdMessage::DRONE_ZCAMERA_OFFSET_NOTIFICATION};
    uint8_t space; // not used
    float offsetX, offsetY, offsetZ;  // Position of the z-camera in the drone's local coordinate system
};

struct DronePositionPose
{
    const AIRT_Message_Header header{StdMessage::DRONE_NOTIFICATIONS_MODULE, StdMessage::DRONE_POSITION_POSE_NOTIFICATION};
    uint8_t space; // now used
    double timestamp; // local clock
    float pitch, roll, yaw; // radians
    float x, y, z; // mm
};

struct DroneGimbalPose
{
    const AIRT_Message_Header header{StdMessage::DRONE_NOTIFICATIONS_MODULE, StdMessage::DRONE_GIMBAL_POSE_NOTIFICATION};
    uint8_t space; // now used
    double timestamp; // local clock
    float pitch, roll, yaw; // radians
};





#pragma pack(pop)