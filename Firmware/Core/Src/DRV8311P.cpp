#include "DRV8311P.h"


uint16_t motor_output_buffer[12];                       // 4 x N Where N is number of motors
static volatile __attribute__((section(".dma_buffer"), aligned(2))) uint16_t tspi_buffer[4];
static uint16_t buffer_index;


void DRV8311P::Config(SPI_HandleTypeDef *spi, ADDRESS address) {
    spi_        = spi;
    addr_       = address << 11;
    if (address==DRV0) {tspi_index_ = 0;}               // Indexes for this project only
    if (address==DRV1) {tspi_index_ = 4;}
    if (address==DRV3) {tspi_index_ = 8;}
}

void DRV8311P::clearFault() {
    tspiWrite16(DRV8311_REG_FLT_CLR, 0x0001);
}

void DRV8311P::clearPWM() {
    tspiWrite16(DRV8311_REG_PWMG_CTRL, 0x0000);
    tspiWrite16(DRV8311_REG_PWM_CTRL1, 0x0007);
    tspiWrite16(DRV8311_REG_PWMG_A_DUTY, 0x0000);
    tspiWrite16(DRV8311_REG_PWMG_B_DUTY, 0x0000);
    tspiWrite16(DRV8311_REG_PWMG_C_DUTY, 0x0000);
    tspiWrite16(DRV8311_REG_PWMG_PERIOD, 0x0000);
    tspiWrite16(DRV8311_REG_PWM_SYNC_PRD, 0x0000);
}

bool DRV8311P::enableOutput() {
    uint16_t data = tspiRead16(DRV8311_REG_PWMG_CTRL);
    tspiWrite16(DRV8311_REG_PWMG_CTRL, ((data & (~PWM_EN_MASK_)) | PWM_EN_MASK_));

    data = tspiRead16(DRV8311_REG_PWMG_CTRL);
    return bool(data & PWM_EN_MASK_);
}

bool DRV8311P::disableOutput() {
    uint16_t data = tspiRead16(DRV8311_REG_PWMG_CTRL);
    tspiWrite16(DRV8311_REG_PWMG_CTRL, (data & (~PWM_EN_MASK_)));
    
    data = tspiRead16(DRV8311_REG_PWMG_CTRL);
    return ~bool(data & PWM_EN_MASK_);
}

void DRV8311P::setOutputPeriod(uint16_t Period) {
    tspiWrite16(DRV8311_REG_PWMG_PERIOD, Period);
}

void DRV8311P::updateOutputDuty(uint16_t PhaseA, uint16_t PhaseB, uint16_t PhaseC) {
    uint16_t data[4] = {uint16_t(addr_ | (DRV8311_REG_PWMG_A_DUTY << 3)), PhaseA, PhaseB, PhaseC};
    HAL_SPI_Transmit(spi_, (uint8_t*)data, 4, HAL_MAX_DELAY);
}

void DRV8311P::updateOutputBuffer(uint16_t PhaseA, uint16_t PhaseB, uint16_t PhaseC) {
    motor_output_buffer[0 + tspi_index_] = (addr_ | (DRV8311_REG_PWMG_A_DUTY << 3));
    motor_output_buffer[1 + tspi_index_] = PhaseA;
    motor_output_buffer[2 + tspi_index_] = PhaseB;
    motor_output_buffer[3 + tspi_index_] = PhaseC;
}

void DRV8311P::setGainCSA(CSA_GAIN Gain) {
    tspiWrite16(DRV8311_REG_CSA_CTRL, uint16_t(Gain));
}

void DRV8311P::setCurrentOCP(OCP_LEVEL Current) {
    uint16_t data = tspiRead16(DRV8311_REG_DRVF_CTRL);
    tspiWrite16(DRV8311_REG_DRVF_CTRL, ((data & (~OCP_LVL_MASK_)) | Current));
}

void DRV8311P::setDeadTime(DEAD_TIME Time) {
    uint16_t data = tspiRead16(DRV8311_REG_DRV_CTRL);
    tspiWrite16(DRV8311_REG_DRV_CTRL, ((data & (~TDEAD_CTRL_MASK_)) | Time));
}

void DRV8311P::setSlewRate(SLEW_RATE SlewRate) {
    uint16_t data = tspiRead16(DRV8311_REG_DRV_CTRL);
    tspiWrite16(DRV8311_REG_DRV_CTRL, ((data & (~SLEW_RATE_MASK_)) | SlewRate));
}

void DRV8311P::setSyncMode(PWM_SYNC SyncMode) {
    uint16_t data = tspiRead16(DRV8311_REG_PWMG_CTRL);
    tspiWrite16(DRV8311_REG_PWMG_CTRL, ((data & (~PWM_OSC_SYNC_MASK_)) | SyncMode));
}

uint16_t DRV8311P::readStatus() {
    return tspiRead16(DRV8311_REG_DEV_STS1);
}

void DRV8311P::sleep() {
    HAL_GPIO_WritePin(MOTOR_NSLEEP_PORT, MOTOR_NSLEEP_PIN, GPIO_PIN_RESET);
}

void DRV8311P::wakeup() {
    HAL_GPIO_WritePin(MOTOR_NSLEEP_PORT, MOTOR_NSLEEP_PIN, GPIO_PIN_SET);
}

uint16_t DRV8311P::spiTransfer16(uint16_t Data) {
    uint16_t result;
    HAL_SPI_TransmitReceive(spi_, (uint8_t*)&Data, (uint8_t*)&result, 1, HAL_MAX_DELAY);
    return result;
}
// 
uint16_t DRV8311P::tspiRead16(uint8_t Address) {
    uint16_t tx[2] = {uint16_t(SPI_READ_MASK_ | addr_ | (Address << 3)), 0};
    uint16_t rx[2];
    HAL_SPI_TransmitReceive(spi_, (uint8_t*)tx, (uint8_t*)rx, 2, HAL_MAX_DELAY);
    return rx[1];
}
// 
void DRV8311P::tspiWrite16(uint8_t Address, uint16_t Data) {
    uint16_t data[2] = {uint16_t(addr_ | (Address << 3)), Data};
    HAL_SPI_Transmit(spi_, (uint8_t*)data, 2, HAL_MAX_DELAY);
}

bool DRV8311P::parityCheck(uint16_t data) {
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return (~data) & 1;
}

bool DRV8311P::Init() {
    HAL_GPIO_WritePin(MOTOR_CS_PORT, MOTOR_CS_PIN, GPIO_PIN_SET);
    wakeup();

    HAL_Delay(10);
    clearFault();
    clearPWM();
    setOutputPeriod(MOTOR_PWM_PERIOD);

    return ~( bool(readStatus() & 0B0000000111100111) );
}

__attribute__((section(".RamFunc")))
void startDMAmotor() {
    for (int i = 0; i < 4; i++) { tspi_buffer[i] = motor_output_buffer[i];}
    HAL_SPI_Transmit_DMA(&hspi4, (uint8_t*)tspi_buffer, 4);
    buffer_index = 1;
}

__attribute__((section(".RamFunc")))
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi4) {
      if (buffer_index < 3) {     // N, where N is the number of motors
        for (int i = 0; i < 4; i++) {tspi_buffer[i] = motor_output_buffer[i + 4*buffer_index];}
        buffer_index += 1;
        HAL_SPI_Transmit_DMA(&hspi4, (uint8_t*)tspi_buffer, 4);
    }                       
  }
}