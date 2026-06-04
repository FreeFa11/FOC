#pragma once


#include "spi.h"
#include "gpio.h"


/*******************************************************************************************************/ 
#define ENCODER_CS_PORT                     SENSOR_NCS1_GPIO_Port   // Chip Select Port
#define ENCODER_CS_PIN                      SENSOR_NCS1_Pin         // Chip Select Pin
// #define ENCODER_CS_PORT_X                   SENSOR_NCS2_GPIO_Port
// #define ENCODER_CS_PORT_Y                   SENSOR_NCS3_GPIO_Port
// #define ENCODER_CS_PORT_Z                   SENSOR_NCS4_GPIO_Port
// #define ENCODER_CS_PIN_X                    SENSOR_NCS2_Pin
// #define ENCODER_CS_PIN_Y                    SENSOR_NCS3_Pin
// #define ENCODER_CS_PIN_Z                    SENSOR_NCS4_Pin
// #define MOTOR_ANGLE_RANGE_DEG               300                     // Physical Rotation Limit
#ifdef MOTOR_ANGLE_RANGE_DEG
#define ENCODER_ANGLE_MIN_CNT_M             1830                    // Measured Minimum Encoder Count
#define ENCODER_ANGLE_MAX_CNT_M             15210                   // Measured Maximum Encoder Count
/*******************************************************************************************************/ 


#define MOTOR_CTRL_MARGIN_DEG               15
#define MOTOR_CTRL_MIN_RAD                  ENCODER_ANGLE_MIN_RAD + MOTOR_CTRL_MARGIN_DEG * _2PI/360.0
#define MOTOR_CTRL_MAX_RAD                  ENCODER_ANGLE_MAX_RAD - MOTOR_CTRL_MARGIN_DEG * _2PI/360.0
#define ENCODER_ANGLE_MIN_CNT               uint32_t((360 - MOTOR_ANGLE_RANGE_DEG) / 720.0f * 16384.0f)
#define ENCODER_ANGLE_MAX_CNT               16383 - ENCODER_ANGLE_MIN_CNT     
#define ENCODER_ANGLE_MIN_RAD               ((360 - MOTOR_ANGLE_RANGE_DEG) / 720.0f * _2PI)
#define ENCODER_ANGLE_MAX_RAD               _2PI - ENCODER_ANGLE_MIN_RAD
#endif

#define _2PI                                6.28318530718f
#define MT6816_CNT_TO_RAD_SCALING           (1 / 16384.0f * _2PI)
#define MT6816_REG_READ_03                  0x83
#define MT6816_REG_READ_04                  0x84
#define MT6816_NO_MAG_WARN_BIT              0x0002




static volatile uint8_t encoder_data[2]; // 2 x N, where N is the number of encoders
static volatile __attribute__((section(".dma_buffer"))) uint8_t rx_mt[2], tx_mt[3];
static uint8_t  buffer_index;
const static uint8_t addr_03[] = {MT6816_REG_READ_03, 0};
const static uint8_t addr_04[] = {MT6816_REG_READ_04, 0};


class MT6816 {
public:
    MT6816(){}
    MT6816(SPI_HandleTypeDef *SPI_Object){spi_ = SPI_Object;}
    virtual ~MT6816();

    void Config(SPI_HandleTypeDef *SPI_Object){spi_ = SPI_Object;};
    virtual void Init();
    uint16_t read();
    float read_rad(){return (read() * MT6816_CNT_TO_RAD_SCALING);};
    void read_buffer();
    float get_rad(){return (position_ * MT6816_CNT_TO_RAD_SCALING);};
    float get_raw(){return position_;};
    
    uint16_t readRaw();
// private:
public:
    int16_t calculateDelta(uint16_t Recent, uint16_t Previous);
    int16_t getInitialPosition(uint16_t Min, uint16_t Max, uint16_t Reading);
    bool parityCheck(uint16_t data);

    SPI_HandleTypeDef *spi_;
    uint16_t counts_=0;
    uint16_t position_=0;
};

__attribute__((section(".RamFunc")))
void startDMAencoder();