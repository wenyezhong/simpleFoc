#include "time_utils.h"
#include "tim.h"

// volatile unsigned int tim_cnt = 0;

void _delay(uint32_t dly)
{
  uint32_t tE = HAL_GetTick()+dly;
	// tim_cnt=dly;
	while(HAL_GetTick()<tE);
}


void delayMicroseconds(uint16_t dly)
{
  uint16_t t = _micros() + dly;
  /* if(t>=50000)
  {
    t -= 50000;
    while(_micros() < 50000){};
    while(_micros() < t){};
  }
  else */
  {
    while( _micros() < t ){};
  }
}


// function buffering delay() 
// arduino uno function doesn't work well with interrupts
/* void _delay(unsigned long ms){

  unsigned long t = _micros() + ms*1000;
  if(t>=50000)
  {
    t -= 50000;
    while(_micros() < 50000){};
    while(_micros() < t){};
  }
  else
  {
    while( _micros() < t ){}; 
  }
}
 */

// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros(){
  uint16_t t= __HAL_TIM_GET_COUNTER(&htim6);
  return t;
}
