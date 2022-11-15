#define __MAIN_CPP__
#include "simpleFoc_main.h"
#include "drv8301.h"
#include <SimpleFOC.h>
 CurrentControl_t current_control_ = {
        .p_gain = 0.0f,        // [V/A] should be auto set after resistance and inductance measurement
        .i_gain = 0.0f,        // [V/As] should be auto set after resistance and inductance measurement
        .v_current_control_integral_d = 0.0f,
        .v_current_control_integral_q = 0.0f,
        .Ibus = 0.0f,
        .final_v_alpha = 0.0f,
        .final_v_beta = 0.0f,
        .Id_setpoint = 0.0f,
        .Iq_setpoint = 0.0f,
        .Iq_measured = 0.0f,
        .Id_measured = 0.0f,
        .I_measured_report_filter_k = 1.0f,
        .max_allowed_current = 0.0f,
        .overcurrent_trip_level = 0.0f,
        .acim_rotor_flux = 0.0f,
        .async_phase_vel = 0.0f,
        .async_phase_offset = 0.0f,
    };
float phase_current_rev_gain_ = 0.0f;
float shunt_conductance = 1.0f / SHUNT_RESISTANCE;
float requested_current_range = 60.0f;


GateDriverHardwareConfig_t gate_driver_config = hw_configs[0].gate_driver_config;
DRV_SPI_8301_Vars_t gate_driver_regs_; //Local view of DRV registers (initialized by DRV8301_setup)
DRV8301_Obj gate_driver_({
            .spiHandle = gate_driver_config.spi,
            .EngpioHandle = gate_driver_config.enable_port,
            .EngpioNumber = gate_driver_config.enable_pin,
            .nCSgpioHandle = gate_driver_config.nCS_port,
            .nCSgpioNumber = gate_driver_config.nCS_pin,
        });

extern "C" {

// void Motor::DRV8301_setup() {
void DRV8301_setup() {
    // for reference:
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A

    // Solve for exact gain, then snap down to have equal or larger range as requested
    // or largest possible range otherwise
    constexpr float kMargin = 0.90f;
    constexpr float kTripMargin = 1.0f; // Trip level is at edge of linear range of amplifer
    constexpr float max_output_swing = 1.35f; // [V] out of amplifier                                                                                                                                              
    float max_unity_gain_current = kMargin * max_output_swing * shunt_conductance; // [A]
    float requested_gain = max_unity_gain_current / requested_current_range; // [V/V]
    printf("max_unity_gain_current=%f\r\n",max_unity_gain_current);
    printf("requested_gain=%f\r\n",requested_gain);
    // Decoding array for snapping gain
    std::array<std::pair<float, DRV8301_ShuntAmpGain_e>, 4> gain_choices = { 
        std::make_pair(10.0f, DRV8301_ShuntAmpGain_10VpV),
        std::make_pair(20.0f, DRV8301_ShuntAmpGain_20VpV),
        std::make_pair(40.0f, DRV8301_ShuntAmpGain_40VpV),
        std::make_pair(80.0f, DRV8301_ShuntAmpGain_80VpV)
    };

    // We use lower_bound in reverse because it snaps up by default, we want to snap down.
    auto gain_snap_down = std::lower_bound(gain_choices.crbegin(), gain_choices.crend(), requested_gain, 
    [](std::pair<float, DRV8301_ShuntAmpGain_e> pair, float val){
        return pair.first > val;
    });
 
    // If we snap to outside the array, clip to smallest val
    if(gain_snap_down == gain_choices.crend())
       --gain_snap_down;

    // Values for current controller
    phase_current_rev_gain_ = 1.0f / gain_snap_down->first;
    printf("pair.first=%f\r\n",gain_snap_down->first);
    printf("phase_current_rev_gain_=%f\r\n",phase_current_rev_gain_);
    // Clip all current control to actual usable range
    current_control_.max_allowed_current = max_unity_gain_current * phase_current_rev_gain_;
    // Set trip level
    current_control_.overcurrent_trip_level = (kTripMargin / kMargin) * current_control_.max_allowed_current;

    printf("max_allowed_current=%f\r\n",current_control_.max_allowed_current);
    printf("overcurrent_trip_level=%f\r\n",current_control_.overcurrent_trip_level);

    // We now have the gain settings we want to use, lets set up DRV chip
    DRV_SPI_8301_Vars_t* local_regs = &gate_driver_regs_;
    DRV8301_enable(&gate_driver_);
    DRV8301_setupSpi(&gate_driver_, local_regs);

    local_regs->Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
    local_regs->Ctrl_Reg_1.PWM_MODE = DRV8301_PwmMode_Three_Inputs; //3 pwm模式
    // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
    local_regs->Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
    local_regs->Ctrl_Reg_2.GAIN = gain_snap_down->second;
    printf("gain_snap_down->second=%d\r\n",gain_snap_down->second);
    //测试drv8301 寄存器读写成功否   
    local_regs->SndCmd = true;
    // printf("Ctrl_Reg_1_Value=%4x\r\n",local_regs->Ctrl_Reg_1);
    printf("GAIN=%4x\r\n",local_regs->Ctrl_Reg_2.GAIN);
    DRV8301_writeData(&gate_driver_, local_regs);
    
    memset(local_regs,0x00,sizeof(gate_driver_regs_));
    printf("GAIN=%4x\r\n",local_regs->Ctrl_Reg_2.GAIN);
    local_regs->RcvCmd = true;    
    
    DRV8301_readData(&gate_driver_, local_regs); 
    printf("OC_MODE=%4x\r\n",gate_driver_regs_.Ctrl_Reg_1.OC_MODE);
    printf("PWM_MODE=%4x\r\n",gate_driver_regs_.Ctrl_Reg_1.PWM_MODE);
    printf("GAIN=%4x\r\n",gate_driver_regs_.Ctrl_Reg_2.GAIN);
    printf("DC_CAL_CH1p2=%4x\r\n",gate_driver_regs_.Ctrl_Reg_2.DC_CAL_CH1p2);
}



Encoder encoder = Encoder(1,2,4096,3);
// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doIndex(){encoder.handleIndex();}

void setup() {
  // monitoring port
//   Serial.begin(115200);

  // enable/disable quadrature mode
  encoder.quadrature = Quadrature::ON;

  // check if you need internal pullups
  encoder.pullup = Pullup::USE_EXTERN;
  
  // initialise encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB,doIndex);

  Serial.println("Encoder ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // not doing much for the encoder though
  encoder.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(encoder.getAngle());
  Serial.print("\t");
  Serial.println(encoder.getVelocity());
}

}