#include "ParameterObject.h"

using eipScanner::cip::CipUint;

const CipUint CLASS = 0xA2;

// ---------------------------------------------------------------------------- //
// ---------------------------- POSSIBLE INSTANCES ---------------------------- //
// ---------------------------------------------------------------------------- //

const CipUint ERROR_CODE_ID = 0x0118;       // The existing error code can be read out with this parameter [ENUM]
const CipUint WARN_CODE_ID = 0x0120;        // The existing warning code can be read out with this parameter. [ENUM]
const CipUint ACTUAL_POS_ID = 0x0230;       // This parameter can be used to read out the current actual position in mm. [FLOAT]
const CipUint ACTUAL_VEL_ID = 0x0238;       // This parameter can be used to read out the current actual speed. [UINT16]
const CipUint GRP_PREHOLD_TIME_ID = 0x0380; // This parameter can be used to read and write the time span for the re-gripping. [UINT16]
const CipUint DEAD_LOAD_KG_ID = 0x03A8;     // The mass of the module can be read out and written with this parameter. [FLOAT]
const CipUint TOOL_CENT_POINT_ID = 0x03B0;  // The tool center point (TCP) of the module can be read out and \
                                                                written with this parameter. [6x FLOAT]
const CipUint CENT_OF_MASS_ID = 0x03B8;     // The center of mass and the mass moments of inertia of the module can be read out \
                                                                and written with this parameter. [6x FLOAT]
const CipUint MODULE_TYPE_ID = 0x0500;      // The module type can be read out with this parameter. [ENUM]
const CipUint WP_LOST_DST_ID = 0x0528;      // This parameter can be used to set the traverse path from which a workpiece loss \
                                                                is detected. [FLOAT]
const CipUint WP_RELEASE_DELTA_ID = 0x0540; // With this parameter the relative position delta between the gripping position and \
                                                                release position can be read out and written. [FLOAT]
const CipUint GRP_POS_MARGIN_ID = 0x0580;   // With this parameter the tolerance value of the workpiece position window \
                                                                can be read and written. [FLOAT]
const CipUint MAX_PHYS_STROKE_ID = 0x0588;  // With this parameter the maximum physical stroke of the module can be read. \
                                                                The parameter value does not take into account any stroke restrictions \
                                                                resulting from the fingers used. [FLOAT]
const CipUint GRP_PREPOS_DELTA_ID = 0x05A8; // With this parameter the relative position delta between the pre-position \
                                                                and gripping position can be read out and written. [FLOAT]
const CipUint MIN_POS_ID = 0x0600;          // The smallest position value that can be approached by the module can be \
                                                                read out and written with this parameter. [FLOAT]
const CipUint MAX_POS_ID = 0x0608;          // The largest position value that can be approached by the module can be \
                                                                read out and written with this parameter. [FLOAT]
const CipUint ZERO_POS_OFS_ID = 0x0610;     // The zero point can be adapted to the application with this parameter. [FLOAT]
const CipUint MIN_VEL_ID = 0x0628;          // The minimum movement/gripping velocity with which the module can be \
                                                                moved can be read out with this parameter. [FLOAT]
const CipUint MAX_VEL_ID = 0x0630;          // The maximum positioning speed with which the module can be moved can \
                                                                read out with this parameter. [FLOAT]
const CipUint MAX_GRP_VEL_ID = 0x0650;      // The maximum gripping velocity with which the module can be moved can \
                                                                read out with this parameter. [FLOAT]
const CipUint MIN_GRP_FORCE_ID = 0x0658;    // The minimum gripping force can be read out with this parameter. [FLOAT]
const CipUint MAX_GRP_FORCE_ID = 0x0660;    // The maximum gripping force can be read out with this parameter. [FLOAT]
const CipUint SERIAL_NO_NUM_ID = 0x1020;    // The serial number of the module can be read out numerically  \
                                                                with this parameter. [CHAR[16]]
const CipUint MAC_ADDR_ID = 0x1138;         // The MAC address of the module can be read out with this parameter. [UINT8]
const CipUint ENABLE_SOFTRESET_ID = 0x1330; // The "Restart" function can be enabled with this parameter. [BOOL]

// ---------------------------------------------------------------------------- //
// ---------------------------- POSSIBLE ATTRIBUTES --------------------------- //
// ---------------------------------------------------------------------------- //

const CipUint NAME_ATTRIBUTE = 0x1;
const CipUint VALUE_ATTRIBUTE = 0x5;

// ---------------------------------------------------------------------------- //
// -------------------------------- OUTPUT DATA ------------------------------- //
// ---------------------------------------------------------------------------- //
const uint8_t SET_0 = 0;
const uint8_t SET_1 = 1;
const int32_t JOG_MODE_NEGATIVE_BIT_POS = 0;
const int32_t JOG_MODE_POSITIVE_BIT_POS = 1;
// const int32_t _ = 2; // reserved
const int32_t RELEASE_WORKPIECE_BIT_POS = 3;
const int32_t GRIP_WORKPIECE_BIT_POS = 4;
const int32_t MOVE_TO_ABSOLUTE_POSITION_BIT_POS = 5;
const int32_t MOVE_TO_RELATIVE_POSITION_BIT_POS = 6;
// const int32_t _ = 7;

const int32_t FAST_STOP_BIT_POS = 0;
const int32_t STOP_BIT_POS = 1;
const int32_t ACKNOWLEDGE_BIT_POS = 2;
const int32_t PREPARE_FOR_SHUTDOWN_BIT_POS = 3;
const int32_t SOFT_RESET_BIT_POS = 4;
const int32_t RELEASE_FOR_MANUAL_BIT_POS = 5;
const int32_t REPEAT_COMMAND_TOGGLE_BIT_POS = 6;
const int32_t GRIP_DIRECTION_BIT_POS = 7;