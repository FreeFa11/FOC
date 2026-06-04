/*
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "MPU6500.h"


static volatile __attribute__((section(".dma_buffer"), aligned(4))) uint8_t rx_imu[15];



void MPU6500::Config(I2C_HandleTypeDef *i2c, const I2cAddr addr) {
  i2c_ = i2c;
  dev_ = addr;
  iface_ = I2C;
}
void MPU6500::Config(SPI_HandleTypeDef *spi, const uint16_t cs_pin, GPIO_TypeDef* gpio_port) {
  spi_ = spi;
  dev_ = cs_pin;
  port_ = gpio_port;
  iface_ = SPI;
}
bool MPU6500::Begin() {
  if (iface_ == SPI) {
    HAL_GPIO_WritePin(port_, dev_, GPIO_PIN_SET);
  }


  /* Select clock source to gyro */
  if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) {
    return false;
  }
  /* Check the WHO AM I byte */
  if (!ReadRegisters(WHOAMI_, sizeof(bytes_rx_), &bytes_rx_)) {
    return false;
  }
  if (bytes_rx_ != WHOAMI_MPU6500_) {
    return false;
  }
  // Default Configuration //
  if (!ConfigAccelRange(ACCEL_RANGE_16G)) {
    return false;
  }
  if (!ConfigGyroRange(GYRO_RANGE_2000DPS)) {
    return false;
  }
  if (!ConfigDLPFBandwidth(DLPF_BANDWIDTH_184HZ)) {
    return false;
  }
  if (!ConfigSRD(0)) {
    return false;
  }
  return true;
}
bool MPU6500::EnableDrdyInt() {
  if (!WriteRegister(INT_PIN_CFG_, INT_PULSE_50US_)) {
    return false;
  }
  if (!WriteRegister(INT_ENABLE_, INT_RAW_RDY_EN_)) {
    return false;
  }
  return true;
}
bool MPU6500::DisableDrdyInt() {
  if (!WriteRegister(INT_ENABLE_, INT_DISABLE_)) {
    return false;
  }
  return true;
}
bool MPU6500::ConfigAccelRange(const AccelRange range) {
  AccelRange requested_accel_range_;
  float requested_accel_scale_;
  switch (range) {
    case ACCEL_RANGE_2G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 2.0f / 32767.5f;
      break;
    }
    case ACCEL_RANGE_4G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 4.0f / 32767.5f;
      break;
    }
    case ACCEL_RANGE_8G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 8.0f / 32767.5f;
      break;
    }
    case ACCEL_RANGE_16G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 16.0f / 32767.5f;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try the requested range */
  if (!WriteRegister(ACCEL_CONFIG_, requested_accel_range_)) {
    return false;
  }
  /* Update stored range and scale */
  accel_range_ = requested_accel_range_;
  accel_scale_ = requested_accel_scale_;
  return true;
}
bool MPU6500::ConfigGyroRange(const GyroRange range) {
  GyroRange requested_gyro_range_;
  float requested_gyro_scale_;
  switch (range) {
    case GYRO_RANGE_250DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 250.0f / 32767.5f;
      break;
    }
    case GYRO_RANGE_500DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 500.0f / 32767.5f;
      break;
    }
    case GYRO_RANGE_1000DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 1000.0f / 32767.5f;
      break;
    }
    case GYRO_RANGE_2000DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 2000.0f / 32767.5f;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try setting the requested range */
  if (!WriteRegister(GYRO_CONFIG_, requested_gyro_range_)) {
    return false;
  }
  /* Update stored range and scale */
  gyro_range_ = requested_gyro_range_;
  gyro_scale_ = requested_gyro_scale_;
  return true;
}
bool MPU6500::ConfigSRD(const uint8_t srd) {
  if (!WriteRegister(SMPLRT_DIV_, srd)) {
    return false;
  }
  srd_ = srd;
  return true;
}
bool MPU6500::ConfigDLPFBandwidth(const DlpfBandwidth dlpf) {
  DlpfBandwidth requested_dlpf_;
  switch (dlpf) {
    case DLPF_BANDWIDTH_184HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_92HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_41HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_20HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_10HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_5HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try setting the DLPF */
  if (!WriteRegister(ACCEL_CONFIG2_, requested_dlpf_)) {
    return false;
  }
  if (!WriteRegister(CONFIG_, requested_dlpf_)) {
    return false;
  }
  /* Update stored DLPF */
  dlpf_bandwidth_ = requested_dlpf_;
  return true;
}
void MPU6500::ConfigAccelBiasX(int16_t Bias){
  axb_ = Bias;
}
void MPU6500::ConfigAccelBiasY(int16_t Bias){
  ayb_ = Bias;
}
void MPU6500::ConfigAccelBiasZ(int16_t Bias){
  azb_ = Bias;
}
void MPU6500::ConfigGyroBiasX(int16_t Bias){
  gxb_ = Bias;
}
void MPU6500::ConfigGyroBiasY(int16_t Bias){
  gyb_ = Bias;
}
void MPU6500::ConfigGyroBiasZ(int16_t Bias){
  gzb_ = Bias;
}
void MPU6500::normalizeAccel() {
  float mag = 1/sqrtf(accel_[0]*accel_[0]+accel_[1]*accel_[1]+accel_[2]*accel_[2]);
  accel_[0] *= mag;
  accel_[1] *= mag;
  accel_[2] *= mag;
}
void MPU6500::readDMAbuffer() {
  for (int i=0; i < sizeof(data_buf_); i++) {
    data_buf_[i] = rx_imu[i];
  }

  // The whole sequence is read at once //
  // Therefore, read -> check if new    //
  new_imu_data_ = (data_buf_[0] & RAW_DATA_RDY_INT_);
  if (!new_imu_data_) {
    return;
  }
  accel_cnts_[0] = static_cast<int16_t>(data_buf_[1])  << 8 | data_buf_[2];
  accel_cnts_[1] = static_cast<int16_t>(data_buf_[3])  << 8 | data_buf_[4];
  accel_cnts_[2] = static_cast<int16_t>(data_buf_[5])  << 8 | data_buf_[6];
  temp_cnts_ =     static_cast<int16_t>(data_buf_[7])  << 8 | data_buf_[8];
  gyro_cnts_[0] =  static_cast<int16_t>(data_buf_[9])  << 8 | data_buf_[10];
  gyro_cnts_[1] =  static_cast<int16_t>(data_buf_[11]) << 8 | data_buf_[12];
  gyro_cnts_[2] =  static_cast<int16_t>(data_buf_[13]) << 8 | data_buf_[14];
  // Bias, Scaling and Polarity Corrections //
  accel_[0] = static_cast<float>(accel_cnts_[0]-axb_) * accel_scale_;
  accel_[1] = static_cast<float>(accel_cnts_[1]-ayb_) * accel_scale_;
  accel_[2] = static_cast<float>(accel_cnts_[2]-azb_) * accel_scale_;
  temp_ =    (static_cast<float>(temp_cnts_) - 21.0f) / TEMP_SCALE_ + 21.0f;
  gyro_[0]  = static_cast<float>(gyro_cnts_[0]-gxb_) * gyro_scale_ * DEG2RAD_;
  gyro_[1]  = static_cast<float>(gyro_cnts_[1]-gyb_) * gyro_scale_ * DEG2RAD_;
  gyro_[2]  = static_cast<float>(gyro_cnts_[2]-gzb_) * gyro_scale_ * DEG2RAD_;
  imu_data_.AX = accel_[0];
  imu_data_.AY = accel_[1];
  imu_data_.AZ = accel_[2];
  imu_data_.GX = gyro_[0];
  imu_data_.GY = gyro_[1];
  imu_data_.GZ = gyro_[2];
}
bool MPU6500::Read() {
  new_imu_data_ = false;
  if (!ReadRegisters(INT_STATUS_, sizeof(data_buf_), data_buf_)) {
    return false;
  }
  // The whole sequence is read at once //
  // Therefore, read -> check if new    //
  new_imu_data_ = (data_buf_[0] & RAW_DATA_RDY_INT_);
  if (!new_imu_data_) {
    return false;
  }
  accel_cnts_[0] = static_cast<int16_t>(data_buf_[1])  << 8 | data_buf_[2];
  accel_cnts_[1] = static_cast<int16_t>(data_buf_[3])  << 8 | data_buf_[4];
  accel_cnts_[2] = static_cast<int16_t>(data_buf_[5])  << 8 | data_buf_[6];
  temp_cnts_ =     static_cast<int16_t>(data_buf_[7])  << 8 | data_buf_[8];
  gyro_cnts_[0] =  static_cast<int16_t>(data_buf_[9])  << 8 | data_buf_[10];
  gyro_cnts_[1] =  static_cast<int16_t>(data_buf_[11]) << 8 | data_buf_[12];
  gyro_cnts_[2] =  static_cast<int16_t>(data_buf_[13]) << 8 | data_buf_[14];
  // Bias, Scaling and Polarity Corrections //
  accel_[0] = static_cast<float>(accel_cnts_[0]-axb_) * accel_scale_;
  accel_[1] = static_cast<float>(accel_cnts_[1]-ayb_) * accel_scale_;
  accel_[2] = static_cast<float>(accel_cnts_[2]-azb_) * accel_scale_;
  temp_ =    (static_cast<float>(temp_cnts_) - 21.0f) / TEMP_SCALE_ + 21.0f;
  gyro_[0]  = static_cast<float>(gyro_cnts_[0]-gxb_) * gyro_scale_ * DEG2RAD_;
  gyro_[1]  = static_cast<float>(gyro_cnts_[1]-gyb_) * gyro_scale_ * DEG2RAD_;
  gyro_[2]  = static_cast<float>(gyro_cnts_[2]-gzb_) * gyro_scale_ * DEG2RAD_;
  imu_data_.AX = accel_[0];
  imu_data_.AY = accel_[1];
  imu_data_.AZ = accel_[2];
  imu_data_.GX = gyro_[0];
  imu_data_.GY = gyro_[1];
  imu_data_.GZ = gyro_[2];
  return true;
}
bool MPU6500::WriteRegister(const uint8_t reg, const uint8_t data) {
  uint8_t ret_val;
  uint8_t to_transmit[] = {reg, data};
  if (iface_ == I2C) {
    HAL_I2C_Master_Transmit(i2c_, dev_ << 1, to_transmit, 2, HAL_MAX_DELAY);
  } else {
    HAL_GPIO_WritePin(port_, dev_, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi_, to_transmit, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(port_, dev_, GPIO_PIN_SET);
  }
  HAL_Delay(10);
  // Checking if the value has been correctly written //
  ReadRegisters(reg, sizeof(ret_val), &ret_val);
  if (data == ret_val) {
    return true;
  } else {
    return false;
  }
}
bool MPU6500::ReadRegisters(uint8_t reg, const uint8_t count, uint8_t * const data) {
  if (!data) {return false;}
  if (iface_ == I2C) {
    if (HAL_I2C_Mem_Read(i2c_,
                        dev_ << 1,     // 7-bit device address shifted
                        reg,
                        I2C_MEMADD_SIZE_8BIT,
                        data,
                        count,
                        HAL_MAX_DELAY) == HAL_OK) {
        return true;
    } else {
        return false;
    }
  } else {
    reg |= SPI_READ_;
    HAL_GPIO_WritePin(port_, dev_, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi_, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi_, data, count, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(port_, dev_, GPIO_PIN_SET);
    return true;
  }
}



// DMA Functions
// void startDMAimu() {
//   HAL_I2C_Mem_Read_DMA(&hi2c2, (MPU6500::I2C_ADDR_PRIM << 1), 0x3A, I2C_MEMADD_SIZE_8BIT, (uint8_t*)rx_imu, 15);
// }
// void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//   if (hi2c == &hi2c2) {
//     // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);    // Debugging */
//     // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);  // Debugging */
//   }
// }
