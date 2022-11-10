#ifndef __ODRIVE_MAIN_H
#define __ODRIVE_MAIN_H

// Note on central include scheme by Samuel:
// there are circular dependencies between some of the header files,
// e.g. the Motor header needs a forward declaration of Axis and vice versa
// so I figured I'd make one main header that takes care of
// the forward declarations and right ordering
// btw this pattern is not so uncommon, for instance IIRC the stdlib uses it too

#ifdef __cplusplus

#include <functional>
#include <limits>
#include <cmath>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <cstring>

/* #include <fibre/protocol.hpp>
#include <communication/interface_usb.h>
#include <communication/interface_i2c.h> */
extern "C" {
#endif

// STM specific includes
#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
// #include <i2c.h> 
// #define ARM_MATH_CM4 // TODO: might change in future board versions
#include <arm_math.h>
// OS includes
#include <cmsis_os.h>






#ifdef __cplusplus
}




// Forward Declarations
/* class Axis;
class Motor;
class ODriveCAN;

constexpr size_t AXIS_COUNT = 2;
extern std::array<Axis*, AXIS_COUNT> axes;
extern ODriveCAN *odCAN; */


// ODrive specific includes
#include <time_utils.h>
#include <pid.h>
#include <lowpass_filter.h>
#include <foc_utils.h>
#include <Sensor.h>
#include <FOCMotor.h>
#include <CurrentSense.h>
#include <Commander.h>
#include <SimpleFOCDebug.h>

//#include <communication.h>




#endif // __cplusplus

#endif /* __ODRIVE_MAIN_H */
