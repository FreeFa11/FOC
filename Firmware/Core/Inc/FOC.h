#pragma once


#include "math.h"
#include "adc.h"
#include "dac.h"
#include "spi.h"
#include "tim.h"
#include "DRV8311P.h"


/*******************************************************************************************************/ 
#define FOC_UPDATE_FREQUENCY        20000               // FOC Update Frequency
#define VEL_SAMPLE_FREQUENCY        2000                // Angular Velocity
#define VEL_CUTOFF_FREQUENCY        200                 // Velocity IIR LPF (Hz)
#define POS_UPDATE_FREQUENCY        150                 // Position Control
#define CSA_REF_VOLTAGE             3                   // Current Sense Reference
#define MOTOR_PTP_RESISTANCE        6                   // Phase to Phase (Ohms)
#define MOTOR_POLE_PAIRS            7                   // Magnetic Pole Pairs
#define VBAT_DIV_RATIO              0.04761904761       // R1 200K & R2 10K 
#define USE_DMA_PWM                                     // DMA (Uncomment) / Blocking (Comment) PWM Update
/*******************************************************************************************************/ 


#define FOC_PERIOD_S        (1.0f / FOC_UPDATE_FREQUENCY)
#define VEL_LOOP_COUNTS     (FOC_UPDATE_FREQUENCY / VEL_SAMPLE_FREQUENCY)
#define VEL_PERIOD_S        (1.0f / VEL_SAMPLE_FREQUENCY)
#define CSA_REF_COUNTS      uint32_t((CSA_REF_VOLTAGE / 3.3) * 4095)
#define CSA_ADC_SCALE       float(3.3 / 65535.0 * 2.0)      // 2.0 comes from CSA Gain i.e. 0.5V/A
#define VBAT_SENSE_SCALE    float(1 / VBAT_DIV_RATIO * 3.3 / 65535.0)




// BUFFER
static volatile __attribute__((section(".dma_buffer"), aligned(4))) uint32_t adc1_buffer[6], adc2_buffer[3];
static uint32_t velocity_counter = 0;


// Data Types
typedef struct AlphaBeta {
    float Alpha=0, Beta=0;
} AlphaBeta;

typedef struct DQ {
    float D=0, Q=0;
} DQ;

typedef struct PhaseVoltage {
    float A=0, B=0, C=0;
} PhaseVoltage;

typedef struct PhaseCurrent {
    float A=0, B=0, C=0;
} PhaseCurrent;


template<class T>
T Clamp(T Data, T maxAbsolute);
template<class T>
T Clamp(T Data, T Min, T Max);
AlphaBeta Clarke(PhaseCurrent senseCurrent);
DQ Park(AlphaBeta Axes, float Angle);
AlphaBeta inversePark(DQ Axes, float Angle);
PhaseVoltage inverseClarke(AlphaBeta Axes);
void startSyncTimer();
void stopSyncTimer();
float readVBat();


class PI
{
private:
    float p_=0, i_=0;
    float integral_limit_=0;
    float control_period_=1;                    // Loop Period (T)
    float e_previous_=0, i_previous_=0;         // Previous Terms

public:
    PI(){}
    ~PI(){}
    void SetGain(float Pgain, float Igain);
    void SetControlPeriod(float Period);
    void SetControlFreq(float Frequency);
    void SetIntegralLimit(float AbsoluteLimit);
    float Update(float Error);
};

class SVPWM
{
private:
    // Needs to be reimplemented to support other Drivers
    // Currently Specific to DRV8311P Driver
    DRV8311P drv_;
    float v_amp_ = 3.7;

public:
    SVPWM(){}
    ~SVPWM(){}
    SVPWM(SPI_HandleTypeDef *spi, DRV8311P::ADDRESS address) : drv_(spi, address){}
    void Config(SPI_HandleTypeDef *spi, DRV8311P::ADDRESS address) {drv_.Config(spi, address);}

    bool Init();
    void updateVDC(float Vmax);
    void enableOutput();
    void disableOutput();
    void updateDuty(PhaseVoltage ReferenceVolts);
    // Affects all devices in tSPI BUS
    void resetALL();     
};

class FOC
{
private:
    SVPWM pwm_;
    PI pi_d_, pi_q_, pi_vel_, pi_pos_;
    
    float v_max_, i_max_;
    float electrical_angle_;
    float electrical_offset_rad_;
    float rotor_angle_=0, rotor_velocity_=0; //h
    float velocity_error_, velocity_correct_;
    float velocity_alpha_ = 1.0f - expf(-2.0f * M_PI * VEL_CUTOFF_FREQUENCY / VEL_SAMPLE_FREQUENCY);
    float position_error_, position_correct_;
    
    AlphaBeta ab_current_;
    DQ dq_current_;
    float q_error_, d_error_;
    float q_correct_, d_correct_;
    DQ dq_voltage_;
    AlphaBeta ab_voltage_;
    PhaseVoltage output_voltage;
    
public:
    FOC(){}
    ~FOC(){}
    FOC(DRV8311P::ADDRESS Address) {pwm_.Config(&hspi4, Address);}
    void Config(DRV8311P::ADDRESS Address) {pwm_.Config(&hspi4, Address);}

    void Init();
    void sleep()  {HAL_GPIO_WritePin(MOTOR_NSLEEP_PORT, MOTOR_NSLEEP_PIN, GPIO_PIN_SET);}
    void wakeup() {HAL_GPIO_WritePin(MOTOR_NSLEEP_PORT, MOTOR_NSLEEP_PIN, GPIO_PIN_RESET);}
    void updateVBAT(float Vbat) {   v_max_ = Vbat*0.5f*1.14f;
                                    i_max_ = v_max_/MOTOR_PTP_RESISTANCE;
                                    pwm_.updateVDC(Vbat);   }
    // Electrical Angle Offset Measured in Radians
    void setOffset(float OffsetRad) {electrical_offset_rad_ = OffsetRad;}
    void writeDuty(PhaseVoltage ReferenceVolts) {pwm_.updateDuty(ReferenceVolts);}
    void setCurrentGainPI(float PGainQ, float IGainQ, float PGainD, float IGainD);
    void setCurrentLimitI(float Max) {pi_d_.SetIntegralLimit(Max);pi_q_.SetIntegralLimit(Max);}
    void RunTorque(PhaseCurrent senseCurrent, float rotorAngle, float IQ_ref, float ID_ref);
    void setVelocityGainPI(float PGain, float IGain) {    pi_vel_.SetGain(PGain, IGain);}
    void setVelocityLimitI(float Max) {pi_vel_.SetIntegralLimit(Max);}
    void RunVelocity(PhaseCurrent senseCurrent, float rotorAngle, float Velocity);
    void setPositionGainPI(float PGain, float IGain) {    pi_pos_.SetGain(PGain, IGain);}
    void setPositionLimitI(float Max) {pi_pos_.SetIntegralLimit(Max);}
    void RunPosition(PhaseCurrent senseCurrent, float rotorAngle, float refAngle);
};




// Specific to this PCB Board and Microcontroller
class CSA
{
private:
    typedef struct CurrentInt {
        int32_t A=0, B=0, C=0;
    } CurrentInt;

    PhaseCurrent currents_[3];
    CurrentInt bias_[3];
    PhaseCurrent previous_[3];  // IIR LPF

public:
    CSA() {}
    ~CSA(){}
    
    void Init();                    
    void calculateCurrent();
    PhaseCurrent getCurrentX_amp() {return currents_[1];}
    PhaseCurrent getCurrentY_amp() {return currents_[2];}
    PhaseCurrent getCurrentZ_amp() {return currents_[0];}
};
