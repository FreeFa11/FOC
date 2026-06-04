/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TSPI_SCK_Pin GPIO_PIN_2
#define TSPI_SCK_GPIO_Port GPIOE
#define MCP_NFAULT_Pin GPIO_PIN_3
#define MCP_NFAULT_GPIO_Port GPIOE
#define MCP_NFAULT_EXTI_IRQn EXTI3_IRQn
#define TSPI_NCS_Pin GPIO_PIN_4
#define TSPI_NCS_GPIO_Port GPIOE
#define TSPI_MISO_Pin GPIO_PIN_5
#define TSPI_MISO_GPIO_Port GPIOE
#define TSPI_MOSI_Pin GPIO_PIN_6
#define TSPI_MOSI_GPIO_Port GPIOE
#define MCP_NSLEEP_Pin GPIO_PIN_13
#define MCP_NSLEEP_GPIO_Port GPIOC
#define SENSE_A1C10_Pin GPIO_PIN_0
#define SENSE_A1C10_GPIO_Port GPIOC
#define SENSE_A1C11_Pin GPIO_PIN_1
#define SENSE_A1C11_GPIO_Port GPIOC
#define SENSE_VBAT_Pin GPIO_PIN_2
#define SENSE_VBAT_GPIO_Port GPIOC
#define SENSE_AUX_Pin GPIO_PIN_3
#define SENSE_AUX_GPIO_Port GPIOC
#define SENSE_A1C16_Pin GPIO_PIN_0
#define SENSE_A1C16_GPIO_Port GPIOA
#define SENSE_A1C17_Pin GPIO_PIN_1
#define SENSE_A1C17_GPIO_Port GPIOA
#define SENSE_A1C14_Pin GPIO_PIN_2
#define SENSE_A1C14_GPIO_Port GPIOA
#define SENSE_A1C15_Pin GPIO_PIN_3
#define SENSE_A1C15_GPIO_Port GPIOA
#define REF_SENSE_Pin GPIO_PIN_4
#define REF_SENSE_GPIO_Port GPIOA
#define BMI088_SCK_Pin GPIO_PIN_5
#define BMI088_SCK_GPIO_Port GPIOA
#define BMI088_MISO_Pin GPIO_PIN_6
#define BMI088_MISO_GPIO_Port GPIOA
#define BMI088_MOSI_Pin GPIO_PIN_7
#define BMI088_MOSI_GPIO_Port GPIOA
#define SENSE_A2C4_Pin GPIO_PIN_4
#define SENSE_A2C4_GPIO_Port GPIOC
#define SENSE_A2C8_Pin GPIO_PIN_5
#define SENSE_A2C8_GPIO_Port GPIOC
#define SENSE_A2C9_Pin GPIO_PIN_0
#define SENSE_A2C9_GPIO_Port GPIOB
#define BMI088_GDR_Pin GPIO_PIN_1
#define BMI088_GDR_GPIO_Port GPIOB
#define PWMOUT_T1C2_Pin GPIO_PIN_11
#define PWMOUT_T1C2_GPIO_Port GPIOE
#define BMI088_ACS_Pin GPIO_PIN_12
#define BMI088_ACS_GPIO_Port GPIOE
#define PWMOUT_T1C3_Pin GPIO_PIN_13
#define PWMOUT_T1C3_GPIO_Port GPIOE
#define PWMOUT_T1C4_Pin GPIO_PIN_14
#define PWMOUT_T1C4_GPIO_Port GPIOE
#define BMI088_ADR_Pin GPIO_PIN_15
#define BMI088_ADR_GPIO_Port GPIOE
#define SENSOR_MISO_Pin GPIO_PIN_14
#define SENSOR_MISO_GPIO_Port GPIOB
#define SENSOR_MOSI_Pin GPIO_PIN_15
#define SENSOR_MOSI_GPIO_Port GPIOB
#define BMI088_GCS_Pin GPIO_PIN_10
#define BMI088_GCS_GPIO_Port GPIOD
#define LED_GREEN_Pin GPIO_PIN_11
#define LED_GREEN_GPIO_Port GPIOD
#define SBUS_INV_Pin GPIO_PIN_14
#define SBUS_INV_GPIO_Port GPIOD
#define SDMMC_CD_Pin GPIO_PIN_9
#define SDMMC_CD_GPIO_Port GPIOC
#define PWMOUT_T1C1_Pin GPIO_PIN_8
#define PWMOUT_T1C1_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWD_SWDIO_Pin GPIO_PIN_13
#define SWD_SWDIO_GPIO_Port GPIOA
#define SWD_SWCLK_Pin GPIO_PIN_14
#define SWD_SWCLK_GPIO_Port GPIOA
#define CAN_SILENT_Pin GPIO_PIN_10
#define CAN_SILENT_GPIO_Port GPIOC
#define SENSOR_SCK_Pin GPIO_PIN_3
#define SENSOR_SCK_GPIO_Port GPIOD
#define SENSOR_NCS1_Pin GPIO_PIN_4
#define SENSOR_NCS1_GPIO_Port GPIOD
#define SENSOR_NCS2_Pin GPIO_PIN_5
#define SENSOR_NCS2_GPIO_Port GPIOD
#define SENSOR_NCS3_Pin GPIO_PIN_6
#define SENSOR_NCS3_GPIO_Port GPIOD
#define SENSOR_NCS4_Pin GPIO_PIN_7
#define SENSOR_NCS4_GPIO_Port GPIOD
#define testPin_Pin GPIO_PIN_5
#define testPin_GPIO_Port GPIOB
#define testPinE0_Pin GPIO_PIN_0
#define testPinE0_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
