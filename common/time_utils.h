#ifndef TIME_UTILS_H
#define TIME_UTILS_H
#include "main.h"
#include "foc_utils.h"

#ifdef __cplusplus
extern "C" {
#endif
/** 
 * Function implementing delay() function in milliseconds 
 * - blocking function
 * - hardware specific

 * @param ms number of milliseconds to wait
 */
void _delay(uint32_t ms);

/** 
 * Function implementing timestamp getting function in microseconds
 * hardware specific
 */
unsigned long _micros();
// void delay(unsigned long dly);
void delayMicroseconds(uint16_t dly);

#ifdef __cplusplus
}
#endif
#endif