#include "FOC.h"



template <class T>
T Clamp(T Data, T maxAbsolute) {
    maxAbsolute = std::abs(maxAbsolute);
    return ((Data>maxAbsolute)?maxAbsolute:(Data<(-maxAbsolute))?(-maxAbsolute):Data);
}
template<class T>
T Clamp(T Data, T Min, T Max) {
    return ((Data>Max)?Max:(Data<Min)?Min:Data);
}

AlphaBeta Clarke(PhaseCurrent senseCurrent) {
    AlphaBeta result;
    result.Alpha = senseCurrent.A*0.66667f - (senseCurrent.B + senseCurrent.C)*0.33333f;
    result.Beta  = (senseCurrent.B - senseCurrent.C)*0.57735f;
    return result;
}
DQ Park(AlphaBeta Axes, float Angle) {
    DQ result;
    float costheta, sintheta;
    sintheta = sinf(Angle); costheta = cosf(Angle);
    result.D = costheta*Axes.Alpha + sintheta*Axes.Beta;
    result.Q = costheta*Axes.Beta  - sintheta*Axes.Alpha;
    return result;
}
AlphaBeta inversePark(DQ Axes, float Angle) {
    AlphaBeta result;
    float costheta, sintheta;
    sintheta = sinf(Angle); costheta = cosf(Angle);
    result.Alpha = costheta*Axes.D - sintheta*Axes.Q;
    result.Beta  = sintheta*Axes.D + costheta*Axes.Q;
    return result;
}
PhaseVoltage inverseClarke(AlphaBeta Axes) {
    PhaseVoltage result;
    result.A =  Axes.Alpha;
    result.B = -Axes.Alpha*0.5f + Axes.Beta*0.86602f;
    result.C = -Axes.Alpha*0.5f - Axes.Beta*0.86602f;
    return result;
}

// Timer to sync ADC and MOTOR
void startSyncTimer() {
    __HAL_TIM_SetAutoreload(&htim4, MOTOR_PWM_PERIOD*2);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, MOTOR_PWM_PERIOD);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, MOTOR_PWM_PERIOD);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}
void stopSyncTimer() {
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
}
// Read Battery Voltage
float readVBat() {
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
    return ((HAL_ADC_GetValue(&hadc3) - 1000) * VBAT_SENSE_SCALE);
}


void PI::SetGain(float Pgain, float Igain) {
    p_ = Pgain;
    i_ = Igain;
}
void PI::SetControlPeriod(float Period) {
    this->control_period_ = Period;
}
void PI::SetControlFreq(float Frequency) {
    this->control_period_ = 1.0f/Frequency;
}
void PI::SetIntegralLimit(float AbsoluteLimit) {
    this->integral_limit_ = AbsoluteLimit;
}
float PI::Update(float Error) {
    i_previous_ = Clamp( i_previous_ + i_*(Error+e_previous_)*0.5f*control_period_, integral_limit_ );
    float output = p_*Error     +     i_previous_ ;
    e_previous_ = Error;
    return output; 
}

// DC Voltage powering Motor Driver
void SVPWM::updateVDC(float Vmax) {
    v_amp_ = Vmax*0.5f;
}
// DRV8311 Settings are initialized Here
bool SVPWM::Init() {
    bool success = drv_.Init();
    drv_.setCurrentOCP(DRV8311P::OCP_LEVEL::A5);    
    drv_.setSlewRate(DRV8311P::SLEW_RATE::V180);
    drv_.setDeadTime(DRV8311P::DEAD_TIME::N600);
    drv_.setGainCSA(DRV8311P::CSA_GAIN::GV5);
    drv_.setSyncMode(DRV8311P::PWM_SYNC::PWM_PRD);
    startSyncTimer();
    return success;
}
void SVPWM::enableOutput() {
    drv_.enableOutput();
}
void SVPWM::disableOutput() {
    drv_.disableOutput();
}
void SVPWM::resetALL() {
    drv_.sleep();
    HAL_Delay(5);
    drv_.wakeup();
}
void SVPWM::updateDuty(PhaseVoltage ReferenceVolts) {

    float vmax = std::max(ReferenceVolts.A, std::max(ReferenceVolts.B, ReferenceVolts.C));
    float vmin = std::min(ReferenceVolts.A, std::min(ReferenceVolts.B, ReferenceVolts.C));
    float avg = (vmax + vmin) * 0.5f;
    ReferenceVolts.A -= avg;
    ReferenceVolts.B -= avg;
    ReferenceVolts.C -= avg;

    uint16_t A = uint16_t(MOTOR_PWM_PERIOD/2 + MOTOR_PWM_PERIOD/2 * (ReferenceVolts.A / v_amp_));
    uint16_t B = uint16_t(MOTOR_PWM_PERIOD/2 + MOTOR_PWM_PERIOD/2 * (ReferenceVolts.B / v_amp_));
    uint16_t C = uint16_t(MOTOR_PWM_PERIOD/2 + MOTOR_PWM_PERIOD/2 * (ReferenceVolts.C / v_amp_));
    A = Clamp(A, uint16_t(0), MOTOR_PWM_PERIOD);
    B = Clamp(B, uint16_t(0), MOTOR_PWM_PERIOD);
    C = Clamp(C, uint16_t(0), MOTOR_PWM_PERIOD);

    #ifdef USE_DMA_PWM
    drv_.updateOutputBuffer(A, B, C); // DMA has to be Started separately
    #else
    drv_.updateOutputDuty(A, B, C); // Blocking Output Update
    #endif
}


void FOC::Init() {
    pi_q_.SetControlFreq(FOC_UPDATE_FREQUENCY);
    pi_d_.SetControlFreq(FOC_UPDATE_FREQUENCY);
    pi_vel_.SetControlFreq(VEL_SAMPLE_FREQUENCY);
    pi_pos_.SetControlFreq(POS_UPDATE_FREQUENCY);
    pwm_.Init();
    pwm_.updateVDC( readVBat() );
    pwm_.enableOutput();
}
void FOC::setCurrentGainPI(float PGainQ, float IGainQ, float PGainD, float IGainD) {
    pi_q_.SetGain(PGainQ, IGainQ);
    pi_d_.SetGain(PGainD, IGainD);
}
void FOC::RunTorque(PhaseCurrent senseCurrent, float rotorAngle, float IQ_ref, float ID_ref) {
    electrical_angle_ = rotorAngle * MOTOR_POLE_PAIRS - electrical_offset_rad_;

    ab_current_ = Clarke(senseCurrent);
    dq_current_ = Park(ab_current_, electrical_angle_);

    q_error_   = IQ_ref - dq_current_.Q;
    d_error_   = ID_ref - dq_current_.D;
    q_correct_ = pi_q_.Update(q_error_);
    d_correct_ = pi_d_.Update(d_error_);

    dq_voltage_.Q = Clamp(q_correct_, v_max_);   // Inv.Park Transform (Performs Rotation Only)
    dq_voltage_.D = Clamp(d_correct_, v_max_);   // Inv.Clarke Transform (Consistent Amplitude)

    ab_voltage_ = inversePark(dq_voltage_, electrical_angle_);
    output_voltage = inverseClarke(ab_voltage_);

    pwm_.updateDuty(output_voltage);
}
void FOC::RunVelocity(PhaseCurrent senseCurrent, float rotorAngle, float Velocity) {

    // This Implementation has drawback of having Velocity Spike as first Iteration

    velocity_counter++;
    if (velocity_counter >= VEL_LOOP_COUNTS) {
        velocity_counter = 0;

        #ifdef MOTOR_ANGLE_RANGE_DEG
        rotor_velocity_ += velocity_alpha_ * ((rotorAngle - rotor_angle_)*VEL_SAMPLE_FREQUENCY - rotor_velocity_);
        #else
        velocity_error_ = rotorAngle - rotor_angle_;        // Delta
        if (velocity_error_ >  3.14159f) {velocity_error_ -= 6.28318f;}
        if (velocity_error_ < -3.14159f) {velocity_error_ += 6.28318f;}
        rotor_velocity_ +=  velocity_alpha_ * (velocity_error_ * VEL_SAMPLE_FREQUENCY - rotor_velocity_);
        #endif
        rotor_angle_ = rotorAngle;

        velocity_error_ = Clamp(Velocity, 150.0f) - rotor_velocity_;  // ~16 RPS
        velocity_correct_ = Clamp(pi_vel_.Update(velocity_error_), i_max_);
    }

    RunTorque(senseCurrent, rotorAngle, velocity_correct_, 0);
}
void FOC::RunPosition(PhaseCurrent senseCurrent, float rotorAngle, float refAngle) {
    position_error_ = refAngle - rotorAngle;
    // Handle the angle Wrap Around
    if      (position_error_ < -M_PI) {position_error_ += 2.0f * M_PI;}
    else if (position_error_ >  M_PI) {position_error_ -= 2.0f * M_PI;}

    position_correct_ = pi_pos_.Update(position_error_);
    RunVelocity(senseCurrent, rotorAngle, position_correct_);
}




// DAC and ADC Initialization
void CSA::Init() {
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, CSA_REF_COUNTS);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_Delay(10);

    startSyncTimer();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, 6);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, 3);
    // Delay before reading Buffer (Important to discard Garbage)
    HAL_Delay(5);
    
    for(int i=0; i<100; i++) {
        bias_[0].A += adc1_buffer[0];
        bias_[0].B += adc1_buffer[1];
        bias_[0].C += adc1_buffer[2];
        bias_[1].A += adc1_buffer[3];
        bias_[1].B += adc1_buffer[4];
        bias_[1].C += adc1_buffer[5];
        bias_[2].A += adc2_buffer[0];
        bias_[2].B += adc2_buffer[1];
        bias_[2].C += adc2_buffer[2];
        
        HAL_Delay(1);
    }

    for (int i=0; i<3; i++) {
        bias_[i].A = bias_[i].A/100;
        bias_[i].B = bias_[i].B/100;
        bias_[i].C = bias_[i].C/100;
    }
}
void CSA::calculateCurrent() {
    currents_[0].A= ((int32_t)adc1_buffer[0] - bias_[0].A) * CSA_ADC_SCALE;
    currents_[0].B= ((int32_t)adc1_buffer[1] - bias_[0].B) * CSA_ADC_SCALE;
    currents_[0].C= ((int32_t)adc1_buffer[2] - bias_[0].C) * CSA_ADC_SCALE;
    currents_[1].A= ((int32_t)adc1_buffer[3] - bias_[1].A) * CSA_ADC_SCALE;
    currents_[1].B= ((int32_t)adc1_buffer[4] - bias_[1].B) * CSA_ADC_SCALE;
    currents_[1].C= ((int32_t)adc1_buffer[5] - bias_[1].C) * CSA_ADC_SCALE;
    currents_[2].A= ((int32_t)adc2_buffer[0] - bias_[2].A) * CSA_ADC_SCALE;
    currents_[2].B= ((int32_t)adc2_buffer[1] - bias_[2].B) * CSA_ADC_SCALE;
    currents_[2].C= ((int32_t)adc2_buffer[2] - bias_[2].C) * CSA_ADC_SCALE;

    for (int i=0; i<3; i++) {   // Filter 220khz
        currents_[i].A = previous_[i].A + 0.0433f*(currents_[i].A - previous_[i].A);
        currents_[i].B = previous_[i].B + 0.0433f*(currents_[i].B - previous_[i].B);
        currents_[i].C = previous_[i].C + 0.0433f*(currents_[i].C - previous_[i].C);
        previous_[i].A = currents_[i].A ;
        previous_[i].B = currents_[i].B ;
        previous_[i].C = currents_[i].C ;
    }

    // float iA, iB, iC;
    // for (int i=0; i<3; i++) {
    //     // Correction from Datasheet (Can be Skipped)
    //     iA =  1.001152f*currents_[i].A - 0.003375f*currents_[i].B - 0.003103f*currents_[i].C;
    //     iB =  0.002369f*currents_[i].A + 1.000665f*currents_[i].B - 0.019126f*currents_[i].C;
    //     iC =  0.001234f*currents_[i].A + 0.001595f*currents_[i].B + 0.998166f*currents_[i].C;
    //     currents_[i].A = iA;
    //     currents_[i].B = iB;
    //     currents_[i].C = iC;
    // }
}
