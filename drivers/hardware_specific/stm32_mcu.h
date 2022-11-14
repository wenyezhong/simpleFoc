#ifndef STM32_DRIVER_MCU_DEF
#define STM32_DRIVER_MCU_DEF
#include "../hardware_api.h"
// #include "adc.h"
// #include <stdbool.h>

#if defined(_STM32_DEF_)

// default pwm parameters
// #define _PWM_RESOLUTION 12 // 12bit
// #define _PWM_RANGE 4095.0 // 2^12 -1 = 4095
// #define _PWM_FREQUENCY 25000 // 25khz
// #define _PWM_FREQUENCY_MAX 50000 // 50khz

#define CURRENT_MEAS_PERIOD ( (float)2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1) / (float)TIM_1_8_CLOCK_HZ )
#define CURRENT_MEAS_HZ ( (float)(TIM_1_8_CLOCK_HZ) / (float)(2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1)) )

#if HW_VERSION_VOLTAGE >= 48
#define VBUS_S_DIVIDER_RATIO 19.0f
#elif HW_VERSION_VOLTAGE == 24
#define VBUS_S_DIVIDER_RATIO 11.0f
#else
#error "unknown board voltage"
#endif

#define ADC_CHANNEL_COUNT 16 

// 6pwm parameters
#define _HARDWARE_6PWM 1
#define _SOFTWARE_6PWM 0
#define _ERROR_6PWM -1


typedef struct STM32DriverParams {
  // HardwareTimer* timers[6] = {NULL};
  uint32_t channels[6];
  long pwm_frequency;
  float dead_zone;
  uint8_t interface_type;
} STM32DriverParams;

// timer synchornisation functions
/* void _stopTimers(HardwareTimer **timers_to_stop, int timer_num=6);
void _startTimers(HardwareTimer **timers_to_start, int timer_num=6); */

/* #ifdef __cplusplus
extern "C" {
#endif
// extern void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc, bool injected);
// extern void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc, bool injected);

#ifdef __cplusplus
}
#endif */


#endif
#endif