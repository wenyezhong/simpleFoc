#include "time_utils.h"
#include "tim.h"

volatile unsigned int tim_cnt = 0;

void delay(unsigned long dly)
{
	tim_cnt=dly;
	while(tim_cnt);
}

void delayMicroseconds(unsigned long dly)
{
  unsigned long t = _micros() + dly;
  if(t>=5000)
  {
    t -= 5000;
    while(_micros() < 5000){};
    while(_micros() < t){};
  }
  else
  {
    while( _micros() < t ){}; 
  }
}


// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms){

  unsigned long t = _micros() + ms*1000;
  if(t>=5000)
  {
    t -= 5000;
    while(_micros() < 5000){};
    while(_micros() < t){};
  }
  else
  {
    while( _micros() < t ){}; 
  }
}


// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros(){
  uint16_t t= __HAL_TIM_GET_COUNTER(&htim6);
  return t;
}
