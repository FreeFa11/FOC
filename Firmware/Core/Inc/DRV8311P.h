#pragma once


#include "spi.h"
#include "gpio.h"
#include "math.h"



// Configuration
#define MOTOR_PWM_FREQUENCY         20000
#define MOTOR_PWM_PERIOD            uint16_t(20000000.0 / (MOTOR_PWM_FREQUENCY * 2.0))
#define MOTOR_PWM_RESOLUTION        uint16_t(ceil( log2( (double) MOTOR_PWM_PERIOD) ))

#define MOTOR_CS_PORT               TSPI_NCS_GPIO_Port
#define MOTOR_CS_PIN                TSPI_NCS_Pin
#define MOTOR_NSLEEP_PORT           MCP_NSLEEP_GPIO_Port
#define MOTOR_NSLEEP_PIN            MCP_NSLEEP_Pin


#define DRV8311_REG_DEV_STS1        0x00  // Device Status 1
#define DRV8311_REG_OT_STS          0x04  // Over Temperature Status
#define DRV8311_REG_SUP_STS         0x05  // Supply Status
#define DRV8311_REG_DRV_STS         0x06  // Driver Status
#define DRV8311_REG_SYS_STS         0x07  // System Status
#define DRV8311_REG_PWM_SYNC_PRD    0x0C  // PWM Sync Period
#define DRV8311_REG_PWMG_PERIOD     0x18  // PWM_GEN Period
#define DRV8311_REG_PWMG_A_DUTY     0x19  // PWM_GEN A Duty
#define DRV8311_REG_PWMG_B_DUTY     0x1A  // PWM_GEN B Duty
#define DRV8311_REG_PWMG_C_DUTY     0x1B  // PWM_GEN C Duty
#define DRV8311_REG_PWM_STATE       0x1C  // PWM State
#define DRV8311_REG_PWMG_CTRL       0x1D  // PWM_GEN Control
#define DRV8311_REG_PWM_CTRL1       0x20  // PWM Control 1
#define DRV8311_REG_FLT_MODE        0x10  // Fault Mode 
#define DRV8311_REG_FLT_TCTRL       0x16  // Fault Timing Control 
#define DRV8311_REG_FLT_CLR         0x17  // Fault Clear 
#define DRV8311_REG_SYSF_CTRL       0x12  // System Fault Control 
#define DRV8311_REG_DRVF_CTRL       0x13  // Driver Fault Control 
#define DRV8311_REG_DRV_CTRL        0x22  // Predriver Control
#define DRV8311_REG_CSA_CTRL        0x23  // CSA Control
#define DRV8311_REG_SYS_CTRL        0x3F  // System Control



class DRV8311P
{
public:
    enum ADDRESS : uint8_t {
        DRV0 = 0x00,
        DRV1 = 0x01,
        DRV2 = 0x02,
        DRV3 = 0x03
    };
    enum CSA_GAIN : uint8_t {
        GV25 = 0x00 | 0x08,
        GV5  = 0x01 | 0x08,
        G1V  = 0x02 | 0x08,
        G2V  = 0x03 | 0x08,
    };
    enum OCP_LEVEL : bool {
        A9   = 0,
        A5   = 1,
    };
    enum DEAD_TIME : uint8_t {
        NONE = 0x00 << 4,
        N200 = 0x01 << 4,
        N400 = 0x02 << 4,
        N600 = 0x03 << 4,
        N800 = 0x04 << 4,
        M1   = 0x05 << 4,
        M1_2 = 0x06 << 4,
        M1_4 = 0x07 << 4,
    };
    enum SLEW_RATE : uint8_t {
        V32  = 0x00,
        V75  = 0x01,
        V180 = 0x02,
        V230 = 0x03,
    };
    enum PWM_SYNC : uint8_t {
        DISABLE  = 0x00 << 5,
        SYNC_PRD = 0x01 << 5,    // PWM_SYNC_PRD indicates period of PWM_SYNC signal and can be used to calibrate PWM period
        PWM_PRD  = 0x02 << 5,    // PWM_SYNC used to set PWM period
        OSC_SYNC = 0x05 << 5,    // PWM_SYNC used for oscillator synchronization (only 20 kHz frequency supported)
        OSC_PWM  = 0x06 << 5,    // PWM_SYNC used for oscillator synchronization and setting PWM period (only 20 kHz frequency supported)
        SPI_CLK  = 0x07 << 5,    // SPI Clock pin SCLK used for oscillator synchronization (Configure SPICLK_FREQ_SYNC)
    };

private:
    GPIO_TypeDef   *nsleep_port_ = MCP_NSLEEP_GPIO_Port,   *cs_port_ = TSPI_NCS_GPIO_Port;
    uint16_t        nsleep_pin_  = MCP_NSLEEP_Pin,          cs_pin_  = TSPI_NCS_Pin;

    SPI_HandleTypeDef *spi_;
    uint16_t addr_;
    uint8_t tspi_index_;
    
    uint16_t spiTransfer16(uint16_t Data);
    uint16_t tspiRead16(uint8_t Address);
    void tspiWrite16(uint8_t Address, uint16_t Data);
    bool parityCheck(uint16_t data);
    
public:
    DRV8311P(){}
    ~DRV8311P(){}
    DRV8311P(SPI_HandleTypeDef *spi, ADDRESS address)  : spi_(spi),addr_(address << 11)
            { if (address==DRV0) {tspi_index_ = 0;} // Indexes for this project only
              if (address==DRV1) {tspi_index_ = 4;}
              if (address==DRV3) {tspi_index_ = 8;} }
    void Config(SPI_HandleTypeDef *spi, ADDRESS address);
    uint16_t readStatus();
    void clearFault();
    void clearPWM();
    void setGainCSA(CSA_GAIN Gain);
    void setCurrentOCP(OCP_LEVEL Current);
    void setDeadTime(DEAD_TIME Time);
    void setSlewRate(SLEW_RATE SlewRate);
    void setSyncMode(PWM_SYNC SyncMode);
    bool enableOutput();
    bool disableOutput();
    void setOutputPeriod(uint16_t);
    void updateOutputDuty(uint16_t PhaseA, uint16_t PhaseB, uint16_t PhaseC);
    void updateOutputBuffer(uint16_t PhaseA, uint16_t PhaseB, uint16_t PhaseC);
    void sleep();   // Affects all motors in tSPI Bus
    void wakeup();  // Affects all motors in tSPI Bus
    bool Init();    // This has been set as needed in this Project

private:
    static constexpr uint16_t SPI_READ_MASK_    = 0x8000;
    static constexpr uint16_t FAULT_MASK_       = 0x0001;
    static constexpr uint16_t SPI_FAULT_MASK_   = 0x0040;
    static constexpr uint16_t OCP_LVL_MASK_     = 0x0001;
    static constexpr uint16_t SLEW_RATE_MASK_   = 0x0003;
    static constexpr uint16_t TDEAD_CTRL_MASK_  = 0x0070;
    static constexpr uint16_t PWM_EN_MASK_      = 0x0400;
    static constexpr uint16_t PWM_OSC_SYNC_MASK_= 0x00E0;
};

__attribute__((section(".RamFunc"))) void startDMAmotor();
extern uint16_t motor_output_buffer[12];