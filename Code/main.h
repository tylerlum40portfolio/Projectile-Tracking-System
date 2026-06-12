/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Common application header and CubeMX-generated pin map.
  *
  * This file contains the STM32 HAL include, Error_Handler prototype, and GPIO
  * pin definitions used by the projectile tracking firmware. Important signals
  * include Pixy2 SPI chip select, laser control, pan/tilt motor direction pins,
  * and motor-driver select pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PIXY_CS_Pin GPIO_PIN_4
#define PIXY_CS_GPIO_Port GPIOA
#define PIXY_SCK_Pin GPIO_PIN_5
#define PIXY_SCK_GPIO_Port GPIOA
#define PIXY_MISO_Pin GPIO_PIN_6
#define PIXY_MISO_GPIO_Port GPIOA
#define PIXY_MOSI_Pin GPIO_PIN_7
#define PIXY_MOSI_GPIO_Port GPIOA
#define PAN_INA_Pin GPIO_PIN_12
#define PAN_INA_GPIO_Port GPIOB
#define PAN_INB_Pin GPIO_PIN_13
#define PAN_INB_GPIO_Port GPIOB
#define TILT_INA_Pin GPIO_PIN_14
#define TILT_INA_GPIO_Port GPIOB
#define TILT_INB_Pin GPIO_PIN_15
#define TILT_INB_GPIO_Port GPIOB
#define LASER_SIGNAL_Pin GPIO_PIN_8
#define LASER_SIGNAL_GPIO_Port GPIOA
#define PAN_SEL0_Pin GPIO_PIN_5
#define PAN_SEL0_GPIO_Port GPIOB
#define TILT_SEL0_Pin GPIO_PIN_6
#define TILT_SEL0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
