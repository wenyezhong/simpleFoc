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
// #define ARM_MATH_CM4 // TODO: might change in future board versions
#include <arm_math.h>
// OS includes
// #include <cmsis_os.h>
// Hardware configuration
#if HW_VERSION_MAJOR == 3
#include "board_config_v3.h"
#else
#error "unknown board version"
#endif






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
 struct CurrentControl_t{
        float p_gain; // [V/A]
        float i_gain; // [V/As]
        float v_current_control_integral_d; // [V]
        float v_current_control_integral_q; // [V]
        float Ibus; // DC bus current [A]
        // Voltage applied at end of cycle:
        float final_v_alpha; // [V]
        float final_v_beta; // [V]
        float Id_setpoint; // [A]
        float Iq_setpoint; // [A]
        float Iq_measured; // [A]
        float Id_measured; // [A]
        float I_measured_report_filter_k;
        float max_allowed_current; // [A]
        float overcurrent_trip_level; // [A]
        float acim_rotor_flux; // [A]
        float async_phase_vel; // [rad/s electrical]
        float async_phase_offset; // [rad electrical]
    };



#endif // __cplusplus

#endif /* __ODRIVE_MAIN_H */
