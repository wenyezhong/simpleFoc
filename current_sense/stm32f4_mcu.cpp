#include "hardware_api.h"
#if 1
// #if defined(STM32F4xx)
#include "../common/foc_utils.h"
#include "../drivers/hardware_api.h"
#include "../drivers/hardware_specific/stm32_mcu.h"
#include "hardware_api.h"
#include "adc.h"
#include "stm32f4xx_ll_tim.h"


#define _ADC_VOLTAGE_F4 3.3f
#define _ADC_RESOLUTION_F4 4096.0f

float adc_voltage_conv;
// array of values of 4 injected channels per adc instance (3)
uint32_t adc_val[3]={0};
// does adc interrupt need a downsample - per adc (3)
bool needs_downsample[3] = {1};
// downsampling variable - per adc (3)
uint8_t tim_downsample[3] = {0};

void* _configureADCLowSide(const void* driver_params, const int pinA, const int pinB, const int pinC){

  /* Stm32CurrentSenseParams* cs_params= new Stm32CurrentSenseParams {
    .pins={(int)NOT_SET,(int)NOT_SET,(int)NOT_SET},
    .adc_voltage_conv = (_ADC_VOLTAGE_F4) / (_ADC_RESOLUTION_F4)
  };
  _adc_gpio_init(cs_params, pinA,pinB,pinC);
  if(_adc_init(cs_params, (STM32DriverParams*)driver_params) != 0) return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
  return cs_params;*/
  adc_voltage_conv = (_ADC_VOLTAGE_F4) / (_ADC_RESOLUTION_F4);
  return 0;
} 


void _driverSyncLowSide(TIM_HandleTypeDef* tim_motorHandle)
{
  
  
  // stop all the timers for the driver
  _stopTimers(tim_motorHandle);

  // if timer has repetition counter - it will downsample using it
  // and it does not need the software downsample
  if( IS_TIM_REPETITION_COUNTER_INSTANCE(tim_motorHandle->Instance) ){
    // adjust the initial timer state such that the trigger 
    //   - for DMA transfer aligns with the pwm peaks instead of throughs.
    //   - for interrupt based ADC transfer 
    //   - only necessary for the timers that have repetition counters
    tim_motorHandle->Instance->CR1 |= TIM_CR1_DIR;
    tim_motorHandle->Instance->CNT =  tim_motorHandle->Instance->ARR;
    // remember that this timer has repetition counter - no need to downasmple
    // needs_downsample[_adcToIndex(cs_params->adc_handle)] = 0;
  }
  // set the trigger output event
  LL_TIM_SetTriggerOutput(tim_motorHandle->Instance, LL_TIM_TRGO_UPDATE);
  // start the adc
   // Enable ADC and interrupts
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    // Warp field stabilize.
    _delay(2);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_EOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_EOC);  

  // restart all the timers of the driver
  _startTimers(tim_motorHandle);
}
  

// function reading an ADC value and returning the read voltage
float _readADCVoltageLowSide(int index)
{
  float value;
  switch(index)
  {
    case 1:{
      value = 0.0f;      
    }break;
    case 2:{
      value = adc_val[1] * adc_voltage_conv; 
    }break;
    case 3:{
      value = adc_val[2] * adc_voltage_conv; 
    }break;
    default:value=0.0f;break;     
  } 
  return value;
}

/* 
extern "C" {
  void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *AdcHandle){
    // calculate the instance
    int adc_index = _adcToIndex(AdcHandle);

    // if the timer han't repetition counter - downsample two times
    if( needs_downsample[adc_index] && tim_downsample[adc_index]++ > 0) {
      tim_downsample[adc_index] = 0;
      return;
    }
    
    adc_val[adc_index][0]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_1);
    adc_val[adc_index][1]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_2);
    adc_val[adc_index][2]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_3);    
  }
} */

// volatile float phB_ADCValue,phC_ADCValue;


// constexpr float adc_full_scale = static_cast<float>(1UL << 12UL);
// constexpr float adc_ref_voltage = 3.3f;
float vbus_voltage = 12.0f;

extern "C" {

void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc, bool injected) {
    // constexpr float voltage_scale = adc_ref_voltage * VBUS_S_DIVIDER_RATIO / adc_full_scale;
    // Only one conversion in sequence, so only rank1
    uint32_t ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    vbus_voltage = ADCValue * adc_voltage_conv;
    // printf("vbus_voltage=%f\r\n",vbus_voltage);
}
// This is the callback from the ADC that we expect after the PWM has triggered an ADC conversion.
// Timing diagram: Firmware/timing_diagram_v3.png
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc, bool injected) {
#define calib_tau 0.2f  //@TOTO make more easily configurable
    constexpr float calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau;

    // Ensure ADCs are expected ones to simplify the logic below
    if (!(hadc == &hadc2 || hadc == &hadc3)) {
        // low_level_fault(Motor::ERROR_ADC_FAILED);
        return;
    };    


    uint32_t ADCValue;
    if (injected) {
        ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    } 
    else
    {
        ADCValue = HAL_ADC_GetValue(hadc);
    }
    // float current = axis.motor_.phase_current_from_adcval(ADCValue);
    // printf("ADCValue=%.4x\r\n",ADCValue);
    // int adcval_bal = (int)ADCValue - (1 << 11);
    /*int adcval_bal = ADCValue;
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * phase_current_rev_gain_;
    float current = shunt_volt * shunt_conductance; */
    // printf("current=%f\r\n",current);
    //return current;
    if(hadc == &hadc2)
    {
      adc_val[1] = ADCValue;
    } 
    else
    {
      adc_val[2] = ADCValue;
    }
}




uint16_t adc_measurements_[ADC_CHANNEL_COUNT] = { 0 };
// @brief Given an adc channel return the measured voltage.
// returns NaN if the channel is not valid.
float get_adc_voltage_channel(uint16_t channel)
{
    if (channel < ADC_CHANNEL_COUNT)
        return ((float)adc_measurements_[channel]) * adc_voltage_conv;
    else
        return 0.0f / 0.0f; // NaN
}

void start_general_purpose_adc() {
    ADC_ChannelConfTypeDef sConfig;

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = ADC_CHANNEL_COUNT;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    // Set up sampling sequence (channel 0 ... channel 15)
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    for (uint32_t channel = 0; channel < ADC_CHANNEL_COUNT; ++channel) {
        sConfig.Channel = channel << ADC_CR1_AWDCH_Pos;
        sConfig.Rank = channel + 1; // rank numbering starts at 1
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            Error_Handler();
    }

    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(adc_measurements_), ADC_CHANNEL_COUNT);
}
}

#endif