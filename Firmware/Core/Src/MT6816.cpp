#include "MT6816.h"


MT6816::~MT6816() {
};

void MT6816::Init() {
    HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    
    HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_RESET);
    #ifdef MOTOR_ANGLE_RANGE_DEG
    counts_ = readRaw();
    position_ = uint16_t(counts_ + getInitialPosition(ENCODER_ANGLE_MIN_CNT_M, ENCODER_ANGLE_MAX_CNT_M, counts_));
    #else
    position_ = readRaw();
    #endif
    HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_SET);

    startDMAencoder();
}

#ifdef MOTOR_ANGLE_RANGE_DEG
int16_t MT6816::getInitialPosition(uint16_t Min, uint16_t Max, uint16_t Reading) {
    if (Min > Max) {
        if (Reading > Min)  {return (ENCODER_ANGLE_MIN_CNT - Min);}
        if (Reading < Max)  {return (ENCODER_ANGLE_MAX_CNT - Max);}
    }
    else if (Min < Max) {
        return (ENCODER_ANGLE_MIN_CNT - Min);
    }
    return 0;
}
#endif

uint16_t MT6816::read() {
    HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_RESET);
    #ifdef MOTOR_ANGLE_RANGE_DEG
    uint16_t counts_new = readRaw();
    #else
    uint16_t position_ = readRaw();
    #endif
    HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_SET);
    
    #ifdef MOTOR_ANGLE_RANGE_DEG
    position_ += calculateDelta(counts_new, counts_);
    counts_ = counts_new;
    #endif
    
    return position_;
}

// Non Blocking Read
void MT6816::read_buffer() {
    #ifdef MOTOR_ANGLE_RANGE_DEG
    uint16_t counts_new = ((encoder_data[0] << 8) | encoder_data[1]) >> 2;
    position_ += calculateDelta(counts_new, counts_);
    counts_ = counts_new;
    #else
    position_ = ((encoder_data[0] << 8) | encoder_data[1]) >> 2;
    #endif
}

int16_t MT6816::calculateDelta(uint16_t Recent, uint16_t Previous) {
    int16_t Delta = Recent - Previous;              // Delta (uint16_t safe)
    if      (Delta > 8192)     {Delta -= 16384;}    // Wrap Around Downward
    else if (Delta < -8192)    {Delta += 16384;}    // Wrap Around Upward
    return Delta;
}

// Blocking Read
uint16_t MT6816::readRaw() {
    uint16_t value;
    uint8_t angle_data[2];

    HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi_, addr_03, angle_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_SET);
    value = (angle_data[1] << 8);
    
    HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi_, addr_04, angle_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_SET);
    value = (value | angle_data[1]) >> 2;

    return value;
}

bool MT6816::parityCheck(uint16_t data) {
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return (~data) & 1;
}



__attribute__((section(".RamFunc")))
void startDMAencoder() {
    buffer_index = 0;
    tx_mt[0] = MT6816_REG_READ_03;
    tx_mt[1] = MT6816_REG_READ_04;
    HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi2, addr_03, (uint8_t*)rx_mt, 3);
}

__attribute__((section(".RamFunc")))
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi2) {
        HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_SET);
        // First recieved bytes is dummy rx[0]!! Copy only rx[1]
        encoder_data[buffer_index] = rx_mt[1];

        buffer_index += 1;
        if (buffer_index < 2) {
            HAL_GPIO_WritePin(ENCODER_CS_PORT, ENCODER_CS_PIN, GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive_DMA(&hspi2, addr_04, (uint8_t*)rx_mt, 2);
        }
    }
}
