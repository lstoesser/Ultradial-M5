// Compile the actual Arduino entry points without copying implementation.
#if defined(CORE_CONTROLLER_CHECK)
#include <gl-ii-CAN.h>
#include "../../examples/CoreS3_Motor_Controller/CoreS3_Motor_Controller.ino"
#else
#include "../../examples/Tab5_Motor_Panel/Tab5_Motor_Panel.ino"
#endif
